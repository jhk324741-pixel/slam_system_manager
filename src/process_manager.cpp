#include "slam_system_manager/process_manager.hpp"

#include <cerrno>
#include <chrono>
#include <climits>
#include <csignal>
#include <cstring>
#include <regex>
#include <stdexcept>
#include <utility>

#include <sys/wait.h>
#include <sys/syscall.h>
#include <unistd.h>

#include "rclcpp/logging.hpp"
#include "yaml-cpp/yaml.h"

namespace slam_system_manager
{
namespace
{

constexpr auto kMonitorInterval = std::chrono::milliseconds(100);
constexpr auto kStopPollInterval = std::chrono::milliseconds(50);
constexpr auto kSigkillWait = std::chrono::seconds(2);
constexpr int kFallbackMaxFileDescriptor = 65536;

void closeInheritedFileDescriptors()
{
#ifdef SYS_close_range
  if (syscall(SYS_close_range, static_cast<unsigned int>(STDERR_FILENO + 1), UINT_MAX, 0) == 0) {
    return;
  }
#endif

  for (int fd = STDERR_FILENO + 1; fd < kFallbackMaxFileDescriptor; ++fd) {
    close(fd);
  }
}

bool validProcessName(const std::string & name)
{
  static const std::regex pattern("^[A-Za-z][A-Za-z0-9_]*$");
  return std::regex_match(name, pattern);
}

template<typename T>
T requiredValue(const YAML::Node & parent, const std::string & key, const std::string & process)
{
  const auto node = parent[key];
  if (!node || node.IsNull()) {
    throw std::runtime_error(
            "Missing process configuration key '" + process + "." + key + "'");
  }
  try {
    return node.as<T>();
  } catch (const YAML::Exception & exception) {
    throw std::runtime_error(
            "Invalid process configuration key '" + process + "." + key + "': " +
            exception.what());
  }
}

}  // namespace

ProcessManager::ProcessManager(const rclcpp::Logger & logger)
: logger_(logger), monitor_thread_(&ProcessManager::monitorLoop, this)
{
}

ProcessManager::~ProcessManager()
{
  stopAll();
  shutdown_.store(true);
  if (monitor_thread_.joinable()) {
    monitor_thread_.join();
  }
}

void ProcessManager::loadConfig(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & exception) {
    throw std::runtime_error("Unable to load process config '" + path + "': " + exception.what());
  }

  const auto process_nodes = root["processes"];
  if (!process_nodes || !process_nodes.IsMap() || process_nodes.size() == 0U) {
    throw std::runtime_error("Process config must contain a non-empty 'processes' map");
  }

  std::map<std::string, ProcessRecord> loaded;
  for (const auto & entry : process_nodes) {
    const auto name = entry.first.as<std::string>();
    const auto node = entry.second;
    if (!validProcessName(name)) {
      throw std::runtime_error("Invalid process name: " + name);
    }
    if (!node.IsMap()) {
      throw std::runtime_error("Process configuration must be a map: " + name);
    }

    ProcessConfig config;
    config.command = requiredValue<std::string>(node, "command", name);
    config.auto_start = requiredValue<bool>(node, "auto_start", name);
    config.sigint_timeout_sec = requiredValue<double>(node, "sigint_timeout_sec", name);
    config.sigterm_timeout_sec = requiredValue<double>(node, "sigterm_timeout_sec", name);
    if (node["working_directory"]) {
      config.working_directory = node["working_directory"].as<std::string>();
    }

    if (config.command.empty()) {
      throw std::runtime_error("Process command must not be empty: " + name);
    }
    if (config.sigint_timeout_sec < 0.0 || config.sigterm_timeout_sec < 0.0) {
      throw std::runtime_error("Process stop timeouts must not be negative: " + name);
    }
    loaded.emplace(name, ProcessRecord{std::move(config), -1, std::nullopt});
  }

  std::lock_guard<std::mutex> lock(mutex_);
  for (auto & entry : processes_) {
    if (refreshProcessLocked(entry.first, entry.second)) {
      throw std::runtime_error("Cannot reload process config while a managed process is running");
    }
  }
  processes_ = std::move(loaded);
  RCLCPP_INFO(logger_, "[PROCESS] Loaded %zu process definitions from %s", processes_.size(), path.c_str());
}

bool ProcessManager::startAutoStartProcesses(std::string * error)
{
  std::vector<std::string> names;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & entry : processes_) {
      if (entry.second.config.auto_start) {
        names.push_back(entry.first);
      }
    }
  }

  bool success = true;
  std::string combined_error;
  for (const auto & name : names) {
    std::string start_error;
    if (!startProcess(name, {}, &start_error)) {
      success = false;
      if (!combined_error.empty()) {
        combined_error += "; ";
      }
      combined_error += start_error;
    }
  }
  setError(error, combined_error);
  return success;
}

bool ProcessManager::startProcess(
  const std::string & name,
  const std::unordered_map<std::string, std::string> & substitutions,
  std::string * error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = processes_.find(name);
  if (it == processes_.end()) {
    setError(error, "Unknown process: " + name);
    return false;
  }

  auto & record = it->second;
  if (refreshProcessLocked(name, record)) {
    setError(error, "Process is already running: " + name);
    return false;
  }

  std::string command;
  try {
    command = expandCommand(record.config.command, substitutions);
  } catch (const std::exception & exception) {
    setError(error, exception.what());
    return false;
  }

  const pid_t pid = fork();
  if (pid < 0) {
    setError(error, "fork() failed for " + name + ": " + std::strerror(errno));
    return false;
  }

  if (pid == 0) {
    if (setpgid(0, 0) != 0) {
      _exit(126);
    }
    if (!record.config.working_directory.empty() &&
      chdir(record.config.working_directory.c_str()) != 0)
    {
      _exit(126);
    }
    closeInheritedFileDescriptors();
    execl("/bin/bash", "bash", "-c", command.c_str(), static_cast<char *>(nullptr));
    _exit(127);
  }

  // The child also calls setpgid. EACCES here only means it reached exec first.
  if (setpgid(pid, pid) != 0 && errno != EACCES) {
    RCLCPP_WARN(
      logger_, "[PROCESS] Unable to confirm process group for %s (pid=%d): %s",
      name.c_str(), pid, std::strerror(errno));
  }
  record.pid = pid;
  record.last_exit_code.reset();
  RCLCPP_INFO(logger_, "[PROCESS] Started %s (pid=%d): %s", name.c_str(), pid, command.c_str());
  return true;
}

bool ProcessManager::stopProcess(const std::string & name, std::string * error)
{
  pid_t pid = -1;
  ProcessConfig config;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = processes_.find(name);
    if (it == processes_.end()) {
      setError(error, "Unknown process: " + name);
      return false;
    }
    if (!refreshProcessLocked(name, it->second)) {
      setError(error, "Process is not running: " + name);
      return false;
    }
    pid = it->second.pid;
    config = it->second.config;
  }

  RCLCPP_INFO(logger_, "[PROCESS] Stopping %s (pid=%d) with SIGINT", name.c_str(), pid);
  signalProcessGroup(pid, SIGINT);
  if (waitForExit(name, pid, config.sigint_timeout_sec)) {
    return true;
  }

  RCLCPP_WARN(logger_, "[PROCESS] %s did not stop after SIGINT; sending SIGTERM", name.c_str());
  signalProcessGroup(pid, SIGTERM);
  if (waitForExit(name, pid, config.sigterm_timeout_sec)) {
    return true;
  }

  RCLCPP_ERROR(logger_, "[PROCESS] %s did not stop after SIGTERM; sending SIGKILL", name.c_str());
  signalProcessGroup(pid, SIGKILL);
  if (waitForExit(name, pid, std::chrono::duration<double>(kSigkillWait).count())) {
    return true;
  }

  setError(error, "Process could not be reaped after SIGKILL: " + name);
  return false;
}

bool ProcessManager::restartProcess(
  const std::string & name,
  const std::unordered_map<std::string, std::string> & substitutions,
  std::string * error)
{
  if (isProcessRunning(name) && !stopProcess(name, error)) {
    return false;
  }
  return startProcess(name, substitutions, error);
}

bool ProcessManager::isProcessRunning(const std::string & name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = processes_.find(name);
  return it != processes_.end() && refreshProcessLocked(name, it->second);
}

std::optional<int> ProcessManager::lastExitCode(const std::string & name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = processes_.find(name);
  if (it == processes_.end()) {
    return std::nullopt;
  }
  refreshProcessLocked(name, it->second);
  return it->second.last_exit_code;
}

std::vector<std::string> ProcessManager::processNames() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> names;
  names.reserve(processes_.size());
  for (const auto & entry : processes_) {
    names.push_back(entry.first);
  }
  return names;
}

void ProcessManager::stopAll()
{
  const auto names = processNames();
  for (const auto & name : names) {
    if (isProcessRunning(name)) {
      std::string error;
      if (!stopProcess(name, &error)) {
        RCLCPP_ERROR(logger_, "[PROCESS] %s", error.c_str());
      }
    }
  }
}

void ProcessManager::setError(std::string * output, const std::string & message)
{
  if (output != nullptr) {
    *output = message;
  }
}

std::string ProcessManager::expandCommand(
  const std::string & command,
  const std::unordered_map<std::string, std::string> & substitutions)
{
  std::string expanded = command;
  for (const auto & substitution : substitutions) {
    const std::string token = "{" + substitution.first + "}";
    std::string::size_type position = 0;
    while ((position = expanded.find(token, position)) != std::string::npos) {
      expanded.replace(position, token.size(), substitution.second);
      position += substitution.second.size();
    }
  }

  static const std::regex unresolved(R"(\{[A-Za-z_][A-Za-z0-9_]*\})");
  std::smatch match;
  if (std::regex_search(expanded, match, unresolved)) {
    throw std::runtime_error("Missing command substitution for token: " + match.str());
  }
  return expanded;
}

int ProcessManager::normalizedExitCode(const int wait_status)
{
  if (WIFEXITED(wait_status)) {
    return WEXITSTATUS(wait_status);
  }
  if (WIFSIGNALED(wait_status)) {
    return 128 + WTERMSIG(wait_status);
  }
  return -1;
}

bool ProcessManager::signalProcessGroup(const pid_t pid, const int signal_number)
{
  if (kill(-pid, signal_number) == 0) {
    return true;
  }
  if (errno == ESRCH) {
    return kill(pid, signal_number) == 0 || errno == ESRCH;
  }
  return false;
}

bool ProcessManager::refreshProcessLocked(const std::string & name, ProcessRecord & record)
{
  if (record.pid <= 0) {
    return false;
  }

  int wait_status = 0;
  const pid_t result = waitpid(record.pid, &wait_status, WNOHANG);
  if (result == 0) {
    return true;
  }
  if (result == record.pid) {
    const int exit_code = normalizedExitCode(wait_status);
    RCLCPP_INFO(
      logger_, "[PROCESS] %s exited (pid=%d, exit_code=%d)",
      name.c_str(), record.pid, exit_code);
    record.pid = -1;
    record.last_exit_code = exit_code;
    return false;
  }
  if (result < 0 && errno == EINTR) {
    return true;
  }
  if (result < 0 && errno == ECHILD) {
    RCLCPP_WARN(logger_, "[PROCESS] Lost child status for %s (pid=%d)", name.c_str(), record.pid);
    record.pid = -1;
    return false;
  }
  return true;
}

bool ProcessManager::waitForExit(
  const std::string & name, const pid_t pid, const double timeout_sec)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(timeout_sec));

  do {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = processes_.find(name);
      if (it == processes_.end() || it->second.pid != pid ||
        !refreshProcessLocked(name, it->second))
      {
        return true;
      }
    }
    std::this_thread::sleep_for(kStopPollInterval);
  } while (std::chrono::steady_clock::now() < deadline);
  return false;
}

void ProcessManager::monitorLoop()
{
  while (!shutdown_.load()) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto & entry : processes_) {
        refreshProcessLocked(entry.first, entry.second);
      }
    }
    std::this_thread::sleep_for(kMonitorInterval);
  }
}

}  // namespace slam_system_manager
