#ifndef SLAM_SYSTEM_MANAGER__LOCALIZATION_ADAPTER_HPP_
#define SLAM_SYSTEM_MANAGER__LOCALIZATION_ADAPTER_HPP_

#include <filesystem>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "slam_system_manager/process_manager.hpp"

namespace slam_system_manager
{

struct LocalizationAdapterConfig
{
  double startup_timeout_sec{20.0};
  double startup_stability_sec{1.0};
  std::vector<std::string> required_topics;
};

class LocalizationAdapter
{
public:
  LocalizationAdapter(
    rclcpp::Node & node, ProcessManager & process_manager,
    const std::string & process_config_path);

  bool start(
    const std::string & map_name, const std::filesystem::path & map_path,
    std::string * error = nullptr);
  bool stop(std::string * error = nullptr);
  bool isRunning();

  const LocalizationAdapterConfig & config() const noexcept;

private:
  static void setError(std::string * output, const std::string & message);
  void loadConfig(const std::string & path);
  bool requiredTopicsAvailable(std::vector<std::string> * missing_topics) const;

  rclcpp::Node & node_;
  ProcessManager & process_manager_;
  LocalizationAdapterConfig config_;
};

}  // namespace slam_system_manager

#endif  // SLAM_SYSTEM_MANAGER__LOCALIZATION_ADAPTER_HPP_
