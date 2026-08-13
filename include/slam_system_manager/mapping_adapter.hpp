#ifndef SLAM_SYSTEM_MANAGER__MAPPING_ADAPTER_HPP_
#define SLAM_SYSTEM_MANAGER__MAPPING_ADAPTER_HPP_

#include <filesystem>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "slam_system_manager/process_manager.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace slam_system_manager
{

struct MappingAdapterConfig
{
  std::string template_config;
  std::string runtime_config_name{"mapping_runtime.yaml"};
  std::string save_service_name{"/map_save"};
  std::string save_service_type{"std_srvs/srv/Trigger"};
  double startup_timeout_sec{10.0};
  double save_timeout_sec{60.0};
};

class MappingAdapter
{
public:
  MappingAdapter(
    rclcpp::Node & node, ProcessManager & process_manager,
    const std::string & process_config_path);

  bool start(
    const std::string & map_name, const std::filesystem::path & map_path,
    std::string * error = nullptr);
  bool stop(std::string * error = nullptr);
  bool saveMap(const std::filesystem::path & map_path, std::string * error = nullptr);
  bool isRunning();
  bool saveServiceAvailable() const;

  const MappingAdapterConfig & config() const noexcept;
  std::filesystem::path prepareRuntimeConfig(
    const std::filesystem::path & map_path) const;

private:
  static void setError(std::string * output, const std::string & message);
  static std::filesystem::path resolveConfigPath(const std::string & path);
  void loadConfig(const std::string & path);

  rclcpp::Node & node_;
  ProcessManager & process_manager_;
  MappingAdapterConfig config_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_client_;
};

}  // namespace slam_system_manager

#endif  // SLAM_SYSTEM_MANAGER__MAPPING_ADAPTER_HPP_
