#ifndef SLAM_SYSTEM_MANAGER__RELOCALIZATION_ADAPTER_HPP_
#define SLAM_SYSTEM_MANAGER__RELOCALIZATION_ADAPTER_HPP_

#include <condition_variable>
#include <filesystem>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "slam_system_manager/config_manager.hpp"

namespace slam_system_manager
{

class RelocalizationAdapter
{
public:
  using InitialPoseCallback =
    std::function<void(const geometry_msgs::msg::PoseWithCovarianceStamped &)>;
  using LocalizationConfirmedCallback = std::function<void()>;

  RelocalizationAdapter(
    rclcpp::Node & node, const SystemConfig & config,
    InitialPoseCallback initial_pose_callback,
    LocalizationConfirmedCallback localization_confirmed_callback);
  ~RelocalizationAdapter();

  RelocalizationAdapter(const RelocalizationAdapter &) = delete;
  RelocalizationAdapter & operator=(const RelocalizationAdapter &) = delete;

  void resetSession();
  void setLocalized(bool localized);
  bool localizationConfirmed() const;
  bool initialPoseConsumerReady() const;
  bool waitForInitialPoseConsumer(double timeout_sec) const;
  bool publishInitialPose(
    geometry_msgs::msg::PoseWithCovarianceStamped pose,
    std::string * error = nullptr);

  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> loadLastPose(
    const std::filesystem::path & map_directory,
    std::string * error = nullptr) const;
  bool requestLastPoseSave(
    const std::filesystem::path & map_directory,
    std::string * error = nullptr);

  static bool validatePose(
    geometry_msgs::msg::PoseWithCovarianceStamped * pose,
    const std::string & expected_frame,
    std::string * error = nullptr);

private:
  struct SaveRequest
  {
    std::filesystem::path map_directory;
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
  };

  static void setError(std::string * output, const std::string & message);
  static void writeLastPose(const SaveRequest & request);
  void handleInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message);
  void handleRawOdometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void handleCorrection(const nav_msgs::msg::Odometry::SharedPtr message);
  void saveWorker();

  rclcpp::Node & node_;
  SystemConfig config_;
  InitialPoseCallback initial_pose_callback_;
  LocalizationConfirmedCallback localization_confirmed_callback_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    localization_pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr localization_odometry_publisher_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_odometry_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr correction_subscription_;

  mutable std::mutex data_mutex_;
  bool localized_{false};
  bool correction_available_{false};
  rclcpp::Time relocalization_started_at_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Odometry correction_;
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> latest_trusted_pose_;

  std::mutex save_mutex_;
  std::condition_variable save_condition_;
  std::optional<SaveRequest> pending_save_;
  bool stop_worker_{false};
  std::thread save_thread_;
};

}  // namespace slam_system_manager

#endif  // SLAM_SYSTEM_MANAGER__RELOCALIZATION_ADAPTER_HPP_
