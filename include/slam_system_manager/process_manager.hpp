#ifndef SLAM_SYSTEM_MANAGER__PROCESS_MANAGER_HPP_
#define SLAM_SYSTEM_MANAGER__PROCESS_MANAGER_HPP_

#include <atomic>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <sys/types.h>

#include "rclcpp/logger.hpp"

namespace slam_system_manager
{

struct ProcessConfig
{
  std::string command;
  std::string working_directory;
  std::unordered_map<std::string, std::string> substitutions;
  bool auto_start{false};
  double sigint_timeout_sec{5.0};
  double sigterm_timeout_sec{2.0};
};

class ProcessManager
{
public:
  explicit ProcessManager(const rclcpp::Logger & logger);
  ~ProcessManager();

  ProcessManager(const ProcessManager &) = delete;
  ProcessManager & operator=(const ProcessManager &) = delete;

  void loadConfig(const std::string & path);
  bool startAutoStartProcesses(
    std::string * error = nullptr,
    const std::vector<std::string> & excluded_processes = {});
  bool startProcess(
    const std::string & name,
    const std::unordered_map<std::string, std::string> & substitutions = {},
    std::string * error = nullptr);
  bool stopProcess(const std::string & name, std::string * error = nullptr);
  bool restartProcess(
    const std::string & name,
    const std::unordered_map<std::string, std::string> & substitutions = {},
    std::string * error = nullptr);
  bool isProcessRunning(const std::string & name);
  std::optional<int> lastExitCode(const std::string & name);
  std::vector<std::string> processNames() const;
  std::vector<std::string> autoStartProcessNames() const;
  void stopAll();

private:
  struct ProcessRecord
  {
    ProcessConfig config;
    pid_t pid{-1};
    std::optional<int> last_exit_code;
  };

  static void setError(std::string * output, const std::string & message);
  static std::string expandCommand(
    const std::string & command,
    const std::unordered_map<std::string, std::string> & substitutions);
  static int normalizedExitCode(int wait_status);
  static bool signalProcessGroup(pid_t pid, int signal_number);

  bool refreshProcessLocked(const std::string & name, ProcessRecord & record);
  bool waitForExit(const std::string & name, pid_t pid, double timeout_sec);
  void monitorLoop();

  rclcpp::Logger logger_;
  mutable std::mutex mutex_;
  std::map<std::string, ProcessRecord> processes_;
  std::atomic<bool> shutdown_{false};
  std::thread monitor_thread_;
};

}  // namespace slam_system_manager

#endif  // SLAM_SYSTEM_MANAGER__PROCESS_MANAGER_HPP_
