#include "slam_system_manager/health_monitor.hpp"

#include <algorithm>
#include <functional>
#include <stdexcept>

#include "yaml-cpp/yaml.h"

namespace slam_system_manager
{
namespace
{

template<typename T>
T requiredValue(const YAML::Node & parent, const std::string & key)
{
  const auto node = parent[key];
  if (!node || node.IsNull()) {
    throw std::runtime_error("Missing sensor configuration key: " + key);
  }
  try {
    return node.as<T>();
  } catch (const YAML::Exception & exception) {
    throw std::runtime_error(
            "Invalid sensor configuration key '" + key + "': " + exception.what());
  }
}

void requirePositive(const double value, const std::string & key)
{
  if (value <= 0.0) {
    throw std::runtime_error("Sensor configuration key '" + key + "' must be greater than zero");
  }
}

}  // namespace

HealthMonitor::HealthMonitor(
  rclcpp::Node & node,
  const std::string & pointcloud_topic,
  const std::string & imu_topic,
  const std::string & config_path)
: config_(loadConfig(config_path)), started_at_(std::chrono::steady_clock::now())
{
  if (pointcloud_topic.empty() || imu_topic.empty()) {
    throw std::runtime_error("Health monitor topics must not be empty");
  }

  sensor_callback_group_ = node.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = sensor_callback_group_;
  const auto sensor_qos = rclcpp::SensorDataQoS();
  pointcloud_subscription_ = node.create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic, sensor_qos,
    std::bind(&HealthMonitor::pointcloudCallback, this, std::placeholders::_1),
    subscription_options);
  imu_subscription_ = node.create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, sensor_qos,
    std::bind(&HealthMonitor::imuCallback, this, std::placeholders::_1),
    subscription_options);

  RCLCPP_INFO(
    node.get_logger(),
    "[SENSOR] Monitoring pointcloud=%s (timeout=%.2fs, min=%.2fHz), "
    "imu=%s (timeout=%.2fs, min=%.2fHz)",
    pointcloud_topic.c_str(), config_.pointcloud_timeout_sec, config_.min_pointcloud_hz,
    imu_topic.c_str(), config_.imu_timeout_sec, config_.min_imu_hz);
}

SensorStatus HealthMonitor::getStatus()
{
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);

  pruneSamples(pointcloud_samples_, now, config_.frequency_window_sec);
  pruneSamples(imu_samples_, now, config_.frequency_window_sec);

  SensorStatus status;
  status.startup_grace_elapsed =
    std::chrono::duration<double>(now - started_at_).count() >= config_.startup_grace_sec;
  status.pointcloud_alive = received_pointcloud_ &&
    std::chrono::duration<double>(now - last_pointcloud_time_).count() <=
    config_.pointcloud_timeout_sec;
  status.imu_alive = received_imu_ &&
    std::chrono::duration<double>(now - last_imu_time_).count() <= config_.imu_timeout_sec;

  status.pointcloud_hz = calculateFrequency(pointcloud_samples_);
  status.imu_hz = calculateFrequency(imu_samples_);
  status.pointcloud_rate_ok = status.pointcloud_hz >= config_.min_pointcloud_hz;
  status.imu_rate_ok = status.imu_hz >= config_.min_imu_hz;
  // Stream timeouts determine readiness. Frequency remains diagnostic data; using a
  // short-window rate as a hard gate causes false failures during transient CPU load.
  status.sensor_ready = status.pointcloud_alive && status.imu_alive;
  return status;
}

const SensorConfig & HealthMonitor::config() const noexcept
{
  return config_;
}

SensorConfig HealthMonitor::loadConfig(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & exception) {
    throw std::runtime_error("Unable to load sensor config '" + path + "': " + exception.what());
  }

  SensorConfig config;
  config.pointcloud_timeout_sec = requiredValue<double>(root, "pointcloud_timeout_sec");
  config.imu_timeout_sec = requiredValue<double>(root, "imu_timeout_sec");
  config.min_pointcloud_hz = requiredValue<double>(root, "min_pointcloud_hz");
  config.min_imu_hz = requiredValue<double>(root, "min_imu_hz");
  config.frequency_window_sec = requiredValue<double>(root, "frequency_window_sec");
  config.startup_grace_sec = requiredValue<double>(root, "startup_grace_sec");
  config.active_failure_grace_sec =
    requiredValue<double>(root, "active_failure_grace_sec");
  config.health_check_hz = requiredValue<double>(root, "health_check_hz");

  requirePositive(config.pointcloud_timeout_sec, "pointcloud_timeout_sec");
  requirePositive(config.imu_timeout_sec, "imu_timeout_sec");
  requirePositive(config.min_pointcloud_hz, "min_pointcloud_hz");
  requirePositive(config.min_imu_hz, "min_imu_hz");
  requirePositive(config.frequency_window_sec, "frequency_window_sec");
  requirePositive(config.startup_grace_sec, "startup_grace_sec");
  requirePositive(config.active_failure_grace_sec, "active_failure_grace_sec");
  requirePositive(config.health_check_hz, "health_check_hz");
  return config;
}

void HealthMonitor::pruneSamples(
  std::deque<SteadyTime> & samples, const SteadyTime now, const double window_sec)
{
  while (!samples.empty() &&
    std::chrono::duration<double>(now - samples.front()).count() > window_sec)
  {
    samples.pop_front();
  }
}

double HealthMonitor::calculateFrequency(const std::deque<SteadyTime> & samples)
{
  if (samples.size() < 2U) {
    return 0.0;
  }
  const double elapsed =
    std::chrono::duration<double>(samples.back() - samples.front()).count();
  return elapsed > 0.0 ? static_cast<double>(samples.size() - 1U) / elapsed : 0.0;
}

void HealthMonitor::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  (void)message;
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  received_pointcloud_ = true;
  last_pointcloud_time_ = now;
  pointcloud_samples_.push_back(now);
  pruneSamples(pointcloud_samples_, now, config_.frequency_window_sec);
}

void HealthMonitor::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr message)
{
  (void)message;
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  received_imu_ = true;
  last_imu_time_ = now;
  imu_samples_.push_back(now);
  pruneSamples(imu_samples_, now, config_.frequency_window_sec);
}

}  // namespace slam_system_manager
