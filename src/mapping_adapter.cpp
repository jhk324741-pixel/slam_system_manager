#include "slam_system_manager/mapping_adapter.hpp"

#include <chrono>
#include <fstream>
#include <future>
#include <stdexcept>
#include <system_error>
#include <unordered_map>

#include <unistd.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
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
    throw std::runtime_error("Missing mapping_adapter configuration key: " + key);
  }
  try {
    return value.as<T>();
  } catch (const YAML::Exception & exception) {
    throw std::runtime_error(
            "Invalid mapping_adapter configuration key '" + key + "': " + exception.what());
  }
}

}  // namespace

MappingAdapter::MappingAdapter(
  rclcpp::Node & node, ProcessManager & process_manager,
  const std::string & process_config_path)
: node_(node), process_manager_(process_manager)
{
  loadConfig(process_config_path);
  save_client_ = node_.create_client<std_srvs::srv::Trigger>(config_.save_service_name);
}

bool MappingAdapter::start(
  const std::string & map_name, const fs::path & map_path, std::string * error)
{
  if (process_manager_.isProcessRunning("mapping")) {
    setError(error, "Mapping process is already running");
    return false;
  }
  if (saveServiceAvailable()) {
    setError(error, "Mapping save service already exists; another mapping node may be running");
    return false;
  }

  fs::path runtime_config;
  try {
    runtime_config = prepareRuntimeConfig(map_path);
  } catch (const std::exception & exception) {
    setError(error, exception.what());
    return false;
  }

  const std::unordered_map<std::string, std::string> substitutions{
    {"map_name", map_name},
    {"map_path", map_path.string()},
    {"config_path", runtime_config.parent_path().string()},
    {"config_file", runtime_config.filename().string()}
  };
  std::string start_error;
  if (!process_manager_.startProcess("mapping", substitutions, &start_error)) {
    setError(error, start_error);
    return false;
  }

  const auto timeout = std::chrono::duration<double>(config_.startup_timeout_sec);
  if (!save_client_->wait_for_service(timeout)) {
    std::string stop_error;
    process_manager_.stopProcess("mapping", &stop_error);
    setError(
      error, "Mapping process started but save service '" + config_.save_service_name +
      "' was not ready within " + std::to_string(config_.startup_timeout_sec) + " seconds");
    return false;
  }
  if (!process_manager_.isProcessRunning("mapping")) {
    const auto exit_code = process_manager_.lastExitCode("mapping");
    setError(
      error, "Mapping process exited during startup (exit_code=" +
      (exit_code ? std::to_string(*exit_code) : std::string("unknown")) + ")");
    return false;
  }
  return true;
}

bool MappingAdapter::stop(std::string * error)
{
  if (!process_manager_.isProcessRunning("mapping")) {
    setError(error, "Mapping process is not running");
    return false;
  }
  return process_manager_.stopProcess("mapping", error);
}

bool MappingAdapter::saveMap(const fs::path & map_path, std::string * error)
{
  if (!process_manager_.isProcessRunning("mapping")) {
    setError(error, "Mapping process is not running");
    return false;
  }
  if (!save_client_->service_is_ready()) {
    setError(error, "Mapping save service is not available: " + config_.save_service_name);
    return false;
  }

  std::error_code filesystem_error;
  fs::remove(map_path, filesystem_error);
  if (filesystem_error) {
    setError(error, "Unable to remove previous map output: " + filesystem_error.message());
    return false;
  }

  const auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = save_client_->async_send_request(request);
  const auto timeout = std::chrono::duration<double>(config_.save_timeout_sec);
  if (future.wait_for(timeout) != std::future_status::ready) {
    setError(
      error, "Mapping save service timed out after " +
      std::to_string(config_.save_timeout_sec) + " seconds");
    return false;
  }

  const auto response = future.get();
  if (!response->success) {
    setError(error, "Mapping save service failed: " + response->message);
    return false;
  }
  if (!fs::is_regular_file(map_path)) {
    setError(error, "Mapping save service succeeded but map file was not created: " + map_path.string());
    return false;
  }
  const auto size = fs::file_size(map_path, filesystem_error);
  if (filesystem_error || size == 0U) {
    setError(error, "Mapping output is empty or unreadable: " + map_path.string());
    return false;
  }
  return true;
}

bool MappingAdapter::isRunning()
{
  return process_manager_.isProcessRunning("mapping");
}

bool MappingAdapter::saveServiceAvailable() const
{
  return save_client_ && save_client_->service_is_ready();
}

const MappingAdapterConfig & MappingAdapter::config() const noexcept
{
  return config_;
}

fs::path MappingAdapter::prepareRuntimeConfig(const fs::path & map_path) const
{
  if (!map_path.is_absolute() || map_path.filename() != "map.pcd") {
    throw std::invalid_argument("Mapping output must be an absolute path ending in map.pcd");
  }
  const auto map_directory = map_path.parent_path();
  if (!fs::is_directory(map_directory)) {
    throw std::runtime_error("Map directory does not exist: " + map_directory.string());
  }

  const auto template_path = resolveConfigPath(config_.template_config);
  YAML::Node root;
  try {
    root = YAML::LoadFile(template_path.string());
  } catch (const YAML::Exception & exception) {
    throw std::runtime_error(
            "Unable to load mapping template '" + template_path.string() + "': " + exception.what());
  }

  auto parameters = root["/**"]["ros__parameters"];
  if (!parameters || !parameters.IsMap()) {
    throw std::runtime_error("Mapping template must contain '/**.ros__parameters'");
  }
  parameters["map_file_path"] = map_path.string();
  parameters["pcd_save"]["pcd_save_en"] = true;

  YAML::Emitter output;
  output << root;
  if (!output.good()) {
    throw std::runtime_error("Unable to serialize mapping runtime configuration");
  }

  const auto runtime_path = map_directory / config_.runtime_config_name;
  const auto temporary_path =
    map_directory / ("." + config_.runtime_config_name + ".tmp." + std::to_string(getpid()));
  {
    std::ofstream stream(temporary_path, std::ios::out | std::ios::trunc);
    if (!stream) {
      throw std::runtime_error("Unable to create mapping runtime config: " + temporary_path.string());
    }
    stream << output.c_str() << '\n';
    stream.flush();
    if (!stream) {
      throw std::runtime_error("Unable to write mapping runtime config: " + temporary_path.string());
    }
  }

  std::error_code error;
  fs::rename(temporary_path, runtime_path, error);
  if (error) {
    fs::remove(temporary_path);
    throw std::runtime_error("Unable to install mapping runtime config: " + error.message());
  }
  return runtime_path;
}

void MappingAdapter::setError(std::string * output, const std::string & message)
{
  if (output != nullptr) {
    *output = message;
  }
}

fs::path MappingAdapter::resolveConfigPath(const std::string & path)
{
  constexpr char package_prefix[] = "package://";
  if (path.rfind(package_prefix, 0) != 0U) {
    const fs::path filesystem_path(path);
    if (!filesystem_path.is_absolute()) {
      throw std::invalid_argument("Mapping template path must be absolute or use package://");
    }
    return filesystem_path;
  }

  const auto package_and_path = path.substr(sizeof(package_prefix) - 1U);
  const auto separator = package_and_path.find('/');
  if (separator == std::string::npos || separator == 0U) {
    throw std::invalid_argument("Invalid package:// mapping template path: " + path);
  }
  const auto package_name = package_and_path.substr(0, separator);
  const auto relative_path = package_and_path.substr(separator + 1U);
  return fs::path(ament_index_cpp::get_package_share_directory(package_name)) / relative_path;
}

void MappingAdapter::loadConfig(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & exception) {
    throw std::runtime_error("Unable to load mapping adapter config: " + std::string(exception.what()));
  }
  const auto mapping = root["mapping_adapter"];
  if (!mapping || !mapping.IsMap()) {
    throw std::runtime_error("Process config must contain a mapping_adapter map");
  }

  config_.template_config = requiredValue<std::string>(mapping, "template_config");
  config_.runtime_config_name = requiredValue<std::string>(mapping, "runtime_config_name");
  config_.startup_timeout_sec = requiredValue<double>(mapping, "startup_timeout_sec");
  const auto save_service = mapping["save_service"];
  if (!save_service || !save_service.IsMap()) {
    throw std::runtime_error("mapping_adapter.save_service must be a map");
  }
  config_.save_service_name = requiredValue<std::string>(save_service, "name");
  config_.save_service_type = requiredValue<std::string>(save_service, "type");
  config_.save_timeout_sec = requiredValue<double>(save_service, "timeout_sec");

  if (config_.runtime_config_name.empty() || fs::path(config_.runtime_config_name).has_parent_path()) {
    throw std::runtime_error("mapping_adapter.runtime_config_name must be a file name");
  }
  if (config_.save_service_name.empty() || config_.save_service_name.front() != '/') {
    throw std::runtime_error("mapping adapter save service name must be absolute");
  }
  if (config_.save_service_type != "std_srvs/srv/Trigger") {
    throw std::runtime_error(
            "Unsupported mapping save service type: " + config_.save_service_type);
  }
  if (config_.startup_timeout_sec <= 0.0 || config_.save_timeout_sec <= 0.0) {
    throw std::runtime_error("Mapping adapter timeouts must be positive");
  }
}

}  // namespace slam_system_manager
