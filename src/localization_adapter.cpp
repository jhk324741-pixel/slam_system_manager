#include "slam_system_manager/localization_adapter.hpp"

#include <chrono>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <unordered_map>

#include "yaml-cpp/yaml.h"

namespace slam_system_manager
{
namespace fs = std::filesystem;
namespace
{

template<typename T>
T requiredValue(const YAML::Node & parent, const std::string & key)
{
  const auto value = parent[key];
  if (!value || value.IsNull()) {
    throw std::runtime_error("Missing localization_adapter configuration key: " + key);
  }
  try {
    return value.as<T>();
  } catch (const YAML::Exception & exception) {
    throw std::runtime_error(
            "Invalid localization_adapter configuration key '" + key + "': " +
            exception.what());
  }
}

std::string joinTopics(const std::vector<std::string> & topics)
{
  std::ostringstream output;
  for (std::size_t index = 0; index < topics.size(); ++index) {
    if (index != 0U) {
      output << ", ";
    }
    output << topics[index];
  }
  return output.str();
}

}  // namespace

LocalizationAdapter::LocalizationAdapter(
  rclcpp::Node & node, ProcessManager & process_manager,
  const std::string & process_config_path)
: node_(node), process_manager_(process_manager)
{
  loadConfig(process_config_path);
}

bool LocalizationAdapter::start(
  const std::string & map_name, const fs::path & map_path, std::string * error)
{
  if (process_manager_.isProcessRunning("localization")) {
    setError(error, "Localization process is already running");
    return false;
  }
  if (!map_path.is_absolute() || map_path.filename() != "map.pcd") {
    setError(error, "Localization map must be an absolute path ending in map.pcd");
    return false;
  }

  std::error_code filesystem_error;
  if (!fs::is_regular_file(map_path, filesystem_error)) {
    setError(error, "Localization map does not exist: " + map_path.string());
    return false;
  }
  const auto map_size = fs::file_size(map_path, filesystem_error);
  if (filesystem_error || map_size == 0U) {
    setError(error, "Localization map is empty or unreadable: " + map_path.string());
    return false;
  }

  const std::unordered_map<std::string, std::string> substitutions{
    {"map_name", map_name},
    {"map_path", map_path.string()}
  };
  std::string start_error;
  if (!process_manager_.startProcess("localization", substitutions, &start_error)) {
    setError(error, start_error);
    return false;
  }

  const auto started_at = std::chrono::steady_clock::now();
  const auto deadline = started_at +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(config_.startup_timeout_sec));
  const auto stable_at = started_at +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(config_.startup_stability_sec));
  std::vector<std::string> missing_topics;

  while (std::chrono::steady_clock::now() < deadline) {
    if (!process_manager_.isProcessRunning("localization")) {
      const auto exit_code = process_manager_.lastExitCode("localization");
      setError(
        error, "Localization process exited during startup (exit_code=" +
        (exit_code ? std::to_string(*exit_code) : std::string("unknown")) + ")");
      return false;
    }

    missing_topics.clear();
    if (std::chrono::steady_clock::now() >= stable_at &&
      requiredTopicsAvailable(&missing_topics))
    {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::string stop_error;
  process_manager_.stopProcess("localization", &stop_error);
  const auto detail = missing_topics.empty() ?
    std::string("process did not remain stable") :
    std::string("missing publishers: ") + joinTopics(missing_topics);
  setError(
    error, "Localization was not ready within " +
    std::to_string(config_.startup_timeout_sec) + " seconds (" + detail + ")");
  return false;
}

bool LocalizationAdapter::stop(std::string * error)
{
  if (!process_manager_.isProcessRunning("localization")) {
    setError(error, "Localization process is not running");
    return false;
  }
  return process_manager_.stopProcess("localization", error);
}

bool LocalizationAdapter::isRunning()
{
  return process_manager_.isProcessRunning("localization");
}

const LocalizationAdapterConfig & LocalizationAdapter::config() const noexcept
{
  return config_;
}

void LocalizationAdapter::setError(std::string * output, const std::string & message)
{
  if (output != nullptr) {
    *output = message;
  }
}

void LocalizationAdapter::loadConfig(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & exception) {
    throw std::runtime_error(
            "Unable to load localization adapter config: " + std::string(exception.what()));
  }

  const auto localization = root["localization_adapter"];
  if (!localization || !localization.IsMap()) {
    throw std::runtime_error("Process config must contain a localization_adapter map");
  }
  config_.startup_timeout_sec =
    requiredValue<double>(localization, "startup_timeout_sec");
  config_.startup_stability_sec =
    requiredValue<double>(localization, "startup_stability_sec");
  config_.required_topics =
    requiredValue<std::vector<std::string>>(localization, "required_topics");

  if (config_.startup_timeout_sec <= 0.0 || config_.startup_stability_sec <= 0.0 ||
    config_.startup_stability_sec > config_.startup_timeout_sec)
  {
    throw std::runtime_error(
            "Localization adapter startup timing must be positive and stability must not exceed timeout");
  }
  for (const auto & topic : config_.required_topics) {
    if (topic.empty() || topic.front() != '/') {
      throw std::runtime_error("Localization required topics must be absolute: " + topic);
    }
  }
}

bool LocalizationAdapter::requiredTopicsAvailable(
  std::vector<std::string> * missing_topics) const
{
  bool available = true;
  for (const auto & topic : config_.required_topics) {
    if (node_.count_publishers(topic) == 0U) {
      available = false;
      if (missing_topics != nullptr) {
        missing_topics->push_back(topic);
      }
    }
  }
  return available;
}

}  // namespace slam_system_manager
