#ifndef SLAM_SYSTEM_MANAGER__LOCALIZATION_QUALITY_MONITOR_HPP_
#define SLAM_SYSTEM_MANAGER__LOCALIZATION_QUALITY_MONITOR_HPP_

#include <cstddef>
#include <functional>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "slam_system_manager/msg/localization_status.hpp"
#include "slam_system_manager/msg/registration_quality.hpp"

namespace slam_system_manager
{

enum class LocalizationQualityState
{
  UNKNOWN,
  LOCALIZED,
  DEGRADED,
  LOST
};

const char * toString(LocalizationQualityState state) noexcept;

struct LocalizationQualityContext
{
  bool localization_active{false};
  bool localization_initialized{false};
  bool localization_process_alive{false};
  bool sensors_healthy{false};
};

struct LocalizationQualityConfig
{
  double publish_hz{5.0};
  double pose_timeout_sec{0.6};
  double correction_timeout_sec{5.0};
  double registration_timeout_sec{5.0};
  double max_position_jump_m{0.6};
  double max_yaw_jump_deg{25.0};
  double max_linear_velocity_mps{3.0};
  double max_angular_velocity_dps{180.0};
  double max_correction_position_jump_m{0.75};
  double max_correction_yaw_jump_deg{30.0};
  double min_registration_fitness{0.70};
  double max_registration_residual_m{0.60};
  std::size_t bad_frames_to_degraded{3U};
  std::size_t bad_frames_to_lost{10U};
  std::size_t good_frames_to_recover{10U};
  double localized_threshold{0.75};
  double degraded_threshold{0.40};
  double update_weight{0.30};
  double motion_weight{0.25};
  double registration_weight{0.30};
  double overlap_weight{0.15};
};

class LocalizationQualityMonitor
{
public:
  using ContextProvider = std::function<LocalizationQualityContext()>;
  using StateChangeCallback = std::function<void(
      LocalizationQualityState, LocalizationQualityState, const std::string &)>;

  LocalizationQualityMonitor(
    rclcpp::Node & node, const std::string & config_path,
    const std::string & localization_odometry_topic,
    const std::string & localization_pose_topic,
    const std::string & correction_topic,
    const std::string & map_frame,
    ContextProvider context_provider,
    StateChangeCallback state_change_callback = {});

  void reset();
  msg::LocalizationStatus status() const;
  const LocalizationQualityConfig & config() const noexcept;

  // Public ingestion/evaluation methods keep the quality rules deterministic and unit-testable.
  void observeOdometry(
    const nav_msgs::msg::Odometry & message, const rclcpp::Time & receipt_time);
  void observePose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & message,
    const rclcpp::Time & receipt_time);
  void observeCorrection(
    const nav_msgs::msg::Odometry & message, const rclcpp::Time & receipt_time);
  void observeRegistration(
    const msg::RegistrationQuality & message, const rclcpp::Time & receipt_time);
  msg::LocalizationStatus evaluate(
    const rclcpp::Time & now, const LocalizationQualityContext & context,
    bool publish = false);

private:
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & orientation);
  static double wrappedAngleDifference(double current, double previous);
  static bool finitePose(const geometry_msgs::msg::Pose & pose);
  static double ageSeconds(const rclcpp::Time & now, const rclcpp::Time & then);
  static float unavailable();

  void loadConfig(const std::string & path);
  void timerCallback();
  void transitionLocked(LocalizationQualityState next, const std::string & reason);

  rclcpp::Node & node_;
  std::string map_frame_;
  LocalizationQualityConfig config_;
  ContextProvider context_provider_;
  StateChangeCallback state_change_callback_;
  rclcpp::Publisher<msg::LocalizationStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr correction_subscription_;
  rclcpp::Subscription<msg::RegistrationQuality>::SharedPtr registration_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable std::mutex mutex_;
  LocalizationQualityState state_{LocalizationQualityState::UNKNOWN};
  msg::LocalizationStatus status_;
  bool have_odometry_{false};
  bool have_pose_{false};
  bool have_correction_{false};
  bool have_registration_{false};
  rclcpp::Time last_odometry_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pose_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_correction_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_registration_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odometry_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_correction_stamp_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Pose last_odometry_pose_;
  geometry_msgs::msg::Pose last_correction_pose_;
  double position_jump_{0.0};
  double yaw_jump_{0.0};
  double linear_velocity_{0.0};
  double angular_velocity_{0.0};
  double correction_position_jump_{0.0};
  double correction_yaw_jump_{0.0};
  bool motion_sample_valid_{false};
  bool motion_sample_fault_{false};
  bool pose_sample_fault_{false};
  bool correction_sample_fault_{false};
  std::string motion_fault_reason_;
  std::string pose_fault_reason_;
  std::string correction_fault_reason_;
  msg::RegistrationQuality registration_;
  std::size_t consecutive_good_frames_{0U};
  std::size_t consecutive_bad_frames_{0U};
  std::size_t consecutive_critical_frames_{0U};
};

}  // namespace slam_system_manager

#endif  // SLAM_SYSTEM_MANAGER__LOCALIZATION_QUALITY_MONITOR_HPP_
