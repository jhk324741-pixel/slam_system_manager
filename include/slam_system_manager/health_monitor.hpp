#ifndef SLAM_SYSTEM_MANAGER__HEALTH_MONITOR_HPP_
#define SLAM_SYSTEM_MANAGER__HEALTH_MONITOR_HPP_

#include <chrono>
#include <deque>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace slam_system_manager
{

struct SensorConfig
{
  double pointcloud_timeout_sec{1.0};
  double imu_timeout_sec{0.5};
  double min_pointcloud_hz{5.0};
  double min_imu_hz{20.0};
  double frequency_window_sec{2.0};
  double startup_grace_sec{3.0};
  double active_failure_grace_sec{5.0};
  double health_check_hz{10.0};
};

struct SensorStatus
{
  bool pointcloud_alive{false};
  bool imu_alive{false};
  double pointcloud_hz{0.0};
  double imu_hz{0.0};
  bool pointcloud_rate_ok{false};
  bool imu_rate_ok{false};
  bool sensor_ready{false};
  bool startup_grace_elapsed{false};
};

class HealthMonitor
{
public:
  HealthMonitor(
    rclcpp::Node & node,
    const std::string & pointcloud_topic,
    const std::string & imu_topic,
    const std::string & config_path);

  SensorStatus getStatus();
  const SensorConfig & config() const noexcept;

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  static SensorConfig loadConfig(const std::string & path);
  static void pruneSamples(
    std::deque<SteadyTime> & samples, SteadyTime now, double window_sec);
  static double calculateFrequency(const std::deque<SteadyTime> & samples);

  void pointcloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  void imuCallback(sensor_msgs::msg::Imu::ConstSharedPtr message);

  mutable std::mutex mutex_;
  SensorConfig config_;
  SteadyTime started_at_;
  SteadyTime last_pointcloud_time_{};
  SteadyTime last_imu_time_{};
  bool received_pointcloud_{false};
  bool received_imu_{false};
  std::deque<SteadyTime> pointcloud_samples_;
  std::deque<SteadyTime> imu_samples_;

  rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

}  // namespace slam_system_manager

#endif  // SLAM_SYSTEM_MANAGER__HEALTH_MONITOR_HPP_
