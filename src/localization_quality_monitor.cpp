#include "slam_system_manager/localization_quality_monitor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

#include "yaml-cpp/yaml.h"

namespace slam_system_manager
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

template<typename T>
T requiredValue(const YAML::Node & parent, const std::string & key)
{
  const auto value = parent[key];
  if (!value || value.IsNull()) {
    throw std::runtime_error("Missing localization quality key: " + key);
  }
  try {
    return value.as<T>();
  } catch (const YAML::Exception & exception) {
    throw std::runtime_error(
            "Invalid localization quality key '" + key + "': " + exception.what());
  }
}

double clamp01(const double value)
{
  return std::clamp(value, 0.0, 1.0);
}

double freshnessScore(const double age, const double timeout)
{
  if (!std::isfinite(age) || age < 0.0 || age >= timeout) {
    return 0.0;
  }
  const double full_score_until = timeout * 0.5;
  if (age <= full_score_until) {
    return 1.0;
  }
  return clamp01((timeout - age) / (timeout - full_score_until));
}

double norm3(const geometry_msgs::msg::Point & first, const geometry_msgs::msg::Point & second)
{
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  const double dz = first.z - second.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::uint32_t saturatingCount(const std::size_t value)
{
  return static_cast<std::uint32_t>(
    std::min<std::size_t>(value, std::numeric_limits<std::uint32_t>::max()));
}

}  // namespace

const char * toString(const LocalizationQualityState state) noexcept
{
  switch (state) {
    case LocalizationQualityState::UNKNOWN: return "UNKNOWN";
    case LocalizationQualityState::LOCALIZED: return "LOCALIZED";
    case LocalizationQualityState::DEGRADED: return "DEGRADED";
    case LocalizationQualityState::LOST: return "LOST";
  }
  return "UNKNOWN";
}

LocalizationQualityMonitor::LocalizationQualityMonitor(
  rclcpp::Node & node, const std::string & config_path,
  const std::string & localization_odometry_topic,
  const std::string & localization_pose_topic,
  const std::string & correction_topic,
  const std::string & map_frame,
  ContextProvider context_provider,
  StateChangeCallback state_change_callback)
: node_(node),
  map_frame_(map_frame),
  context_provider_(std::move(context_provider)),
  state_change_callback_(std::move(state_change_callback))
{
  loadConfig(config_path);
  status_.state = toString(LocalizationQualityState::UNKNOWN);
  status_.matching_score = unavailable();
  status_.inlier_ratio = unavailable();
  status_.map_overlap_ratio = unavailable();
  status_.registration_residual = unavailable();
  status_.registration_age = unavailable();
  status_.update_age = unavailable();
  status_.reason = "Localization has not been initialized";

  status_publisher_ = node_.create_publisher<msg::LocalizationStatus>(
    "/localization/status", 10);
  odometry_subscription_ = node_.create_subscription<nav_msgs::msg::Odometry>(
    localization_odometry_topic, 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr message) {
      observeOdometry(*message, node_.now());
    });
  pose_subscription_ =
    node_.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    localization_pose_topic, 10,
    [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
      observePose(*message, node_.now());
    });
  correction_subscription_ = node_.create_subscription<nav_msgs::msg::Odometry>(
    correction_topic, 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr message) {
      observeCorrection(*message, node_.now());
    });
  registration_subscription_ = node_.create_subscription<msg::RegistrationQuality>(
    "/localization/registration_quality", 10,
    [this](const msg::RegistrationQuality::SharedPtr message) {
      observeRegistration(*message, node_.now());
    });

  const auto period = std::chrono::duration<double>(1.0 / config_.publish_hz);
  timer_ = node_.create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&LocalizationQualityMonitor::timerCallback, this));
}

void LocalizationQualityMonitor::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = LocalizationQualityState::UNKNOWN;
  status_ = msg::LocalizationStatus{};
  status_.state = toString(state_);
  status_.matching_score = unavailable();
  status_.inlier_ratio = unavailable();
  status_.map_overlap_ratio = unavailable();
  status_.registration_residual = unavailable();
  status_.registration_age = unavailable();
  status_.update_age = unavailable();
  status_.reason = "Localization has not been initialized";
  have_odometry_ = false;
  have_pose_ = false;
  have_correction_ = false;
  have_registration_ = false;
  motion_sample_valid_ = false;
  motion_sample_fault_ = false;
  pose_sample_fault_ = false;
  correction_sample_fault_ = false;
  motion_fault_reason_.clear();
  pose_fault_reason_.clear();
  correction_fault_reason_.clear();
  position_jump_ = 0.0;
  yaw_jump_ = 0.0;
  linear_velocity_ = 0.0;
  angular_velocity_ = 0.0;
  correction_position_jump_ = 0.0;
  correction_yaw_jump_ = 0.0;
  consecutive_good_frames_ = 0U;
  consecutive_bad_frames_ = 0U;
  consecutive_critical_frames_ = 0U;
}

msg::LocalizationStatus LocalizationQualityMonitor::status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

const LocalizationQualityConfig & LocalizationQualityMonitor::config() const noexcept
{
  return config_;
}

void LocalizationQualityMonitor::observeOdometry(
  const nav_msgs::msg::Odometry & message, const rclcpp::Time & receipt_time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!finitePose(message.pose.pose)) {
    motion_sample_fault_ = true;
    motion_fault_reason_ = "Localization odometry contains NaN, Inf, or an invalid quaternion";
    return;
  }

  const rclcpp::Time stamp(message.header.stamp, RCL_ROS_TIME);
  if (have_odometry_ && stamp <= last_odometry_stamp_) {
    motion_sample_fault_ = true;
    motion_fault_reason_ = "Localization odometry timestamp moved backwards or repeated";
    return;
  }

  motion_sample_fault_ = false;
  motion_fault_reason_.clear();
  motion_sample_valid_ = false;
  if (have_odometry_) {
    const double delta_time = (stamp - last_odometry_stamp_).seconds();
    position_jump_ = norm3(message.pose.pose.position, last_odometry_pose_.position);
    yaw_jump_ = std::abs(wrappedAngleDifference(
        yawFromQuaternion(message.pose.pose.orientation),
        yawFromQuaternion(last_odometry_pose_.orientation)));
    if (delta_time > 0.0 && std::isfinite(delta_time)) {
      linear_velocity_ = position_jump_ / delta_time;
      angular_velocity_ = yaw_jump_ / delta_time;
      motion_sample_valid_ = true;
      const bool position_fault =
        position_jump_ > config_.max_position_jump_m &&
        linear_velocity_ > config_.max_linear_velocity_mps;
      const bool yaw_fault =
        yaw_jump_ > config_.max_yaw_jump_deg * kPi / 180.0 &&
        angular_velocity_ > config_.max_angular_velocity_dps * kPi / 180.0;
      if (position_fault) {
        motion_sample_fault_ = true;
        motion_fault_reason_ = "Position jump and linear velocity exceeded thresholds";
      } else if (yaw_fault) {
        motion_sample_fault_ = true;
        motion_fault_reason_ = "Yaw jump and angular velocity exceeded thresholds";
      } else if (linear_velocity_ > config_.max_linear_velocity_mps) {
        motion_sample_fault_ = true;
        motion_fault_reason_ = "Linear velocity exceeded the configured physical limit";
      } else if (angular_velocity_ > config_.max_angular_velocity_dps * kPi / 180.0) {
        motion_sample_fault_ = true;
        motion_fault_reason_ = "Angular velocity exceeded the configured physical limit";
      }
    }
  }

  last_odometry_pose_ = message.pose.pose;
  last_odometry_stamp_ = stamp;
  last_odometry_receipt_ = receipt_time;
  have_odometry_ = true;
}

void LocalizationQualityMonitor::observePose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & message,
  const rclcpp::Time & receipt_time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!finitePose(message.pose.pose)) {
    pose_sample_fault_ = true;
    pose_fault_reason_ = "Localization pose contains NaN, Inf, or an invalid quaternion";
    return;
  }
  pose_sample_fault_ = false;
  pose_fault_reason_.clear();
  last_pose_receipt_ = receipt_time;
  have_pose_ = true;
}

void LocalizationQualityMonitor::observeCorrection(
  const nav_msgs::msg::Odometry & message, const rclcpp::Time & receipt_time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!finitePose(message.pose.pose)) {
    correction_sample_fault_ = true;
    correction_fault_reason_ = "map_to_odom contains NaN, Inf, or an invalid quaternion";
    return;
  }
  const rclcpp::Time stamp(message.header.stamp, RCL_ROS_TIME);
  if (have_correction_ && stamp <= last_correction_stamp_) {
    correction_sample_fault_ = true;
    correction_fault_reason_ = "map_to_odom timestamp moved backwards or repeated";
    return;
  }

  correction_sample_fault_ = false;
  correction_fault_reason_.clear();
  if (have_correction_) {
    correction_position_jump_ = norm3(
      message.pose.pose.position, last_correction_pose_.position);
    correction_yaw_jump_ = std::abs(wrappedAngleDifference(
        yawFromQuaternion(message.pose.pose.orientation),
        yawFromQuaternion(last_correction_pose_.orientation)));
    if (correction_position_jump_ > config_.max_correction_position_jump_m) {
      correction_sample_fault_ = true;
      correction_fault_reason_ = "map_to_odom position correction jumped beyond the threshold";
    } else if (correction_yaw_jump_ > config_.max_correction_yaw_jump_deg * kPi / 180.0) {
      correction_sample_fault_ = true;
      correction_fault_reason_ = "map_to_odom yaw correction jumped beyond the threshold";
    }
  }
  last_correction_pose_ = message.pose.pose;
  last_correction_stamp_ = stamp;
  last_correction_receipt_ = receipt_time;
  have_correction_ = true;
}

void LocalizationQualityMonitor::observeRegistration(
  const msg::RegistrationQuality & message, const rclcpp::Time & receipt_time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  registration_ = message;
  if (registration_.valid &&
    (!std::isfinite(registration_.fitness) ||
    !std::isfinite(registration_.mean_residual) ||
    registration_.source_point_count == 0U ||
    registration_.correspondence_count > registration_.source_point_count))
  {
    registration_.valid = false;
    registration_.accepted = false;
  }
  last_registration_receipt_ = receipt_time;
  have_registration_ = true;
}

msg::LocalizationStatus LocalizationQualityMonitor::evaluate(
  const rclcpp::Time & now, const LocalizationQualityContext & context, const bool publish)
{
  std::lock_guard<std::mutex> lock(mutex_);
  status_.header.stamp = now;
  status_.header.frame_id = map_frame_;
  status_.map_overlap_ratio = unavailable();

  if (!context.localization_active) {
    transitionLocked(LocalizationQualityState::UNKNOWN, "Localization is not active");
    consecutive_good_frames_ = 0U;
    consecutive_bad_frames_ = 0U;
    consecutive_critical_frames_ = 0U;
    status_.confidence = 0.0F;
    status_.pose_valid = false;
    status_.reason = "Localization is not active";
  } else if (!context.localization_initialized) {
    transitionLocked(LocalizationQualityState::UNKNOWN, "Waiting for localization initialization");
    consecutive_good_frames_ = 0U;
    consecutive_bad_frames_ = 0U;
    consecutive_critical_frames_ = 0U;
    status_.confidence = 0.0F;
    status_.pose_valid = false;
    status_.reason = "Waiting for localization initialization";
  } else {
    const double odometry_age = have_odometry_ ?
      ageSeconds(now, last_odometry_receipt_) : std::numeric_limits<double>::quiet_NaN();
    const double pose_age = have_pose_ ?
      ageSeconds(now, last_pose_receipt_) : std::numeric_limits<double>::quiet_NaN();
    const double update_age = (have_odometry_ && have_pose_) ?
      std::max(odometry_age, pose_age) : std::numeric_limits<double>::quiet_NaN();
    const double correction_age = have_correction_ ?
      ageSeconds(now, last_correction_receipt_) : std::numeric_limits<double>::quiet_NaN();
    const double registration_age = have_registration_ ?
      ageSeconds(now, last_registration_receipt_) : std::numeric_limits<double>::quiet_NaN();

    status_.update_age = static_cast<float>(update_age);
    status_.position_jump = static_cast<float>(position_jump_);
    status_.yaw_jump = static_cast<float>(yaw_jump_);
    status_.linear_velocity = static_cast<float>(linear_velocity_);
    status_.angular_velocity = static_cast<float>(angular_velocity_);
    status_.registration_age = static_cast<float>(registration_age);
    status_.matching_score = have_registration_ && registration_.valid ?
      registration_.fitness : unavailable();
    status_.inlier_ratio = have_registration_ && registration_.valid ?
      registration_.fitness : unavailable();
    status_.registration_residual = have_registration_ && registration_.valid ?
      registration_.mean_residual : unavailable();
    status_.correspondence_count = have_registration_ ?
      registration_.correspondence_count : 0U;

    bool critical = false;
    std::string reason = "Localization quality is healthy";
    if (!context.localization_process_alive) {
      critical = true;
      reason = "Localization process is not running";
    } else if (!context.sensors_healthy) {
      critical = true;
      reason = "Ouster data is not healthy";
    } else if (!have_odometry_ || !have_pose_) {
      critical = true;
      reason = "Waiting for unified localization pose updates";
    } else if (!std::isfinite(update_age) || update_age < 0.0) {
      critical = true;
      reason = "Localization update time is invalid";
    } else if (update_age > config_.pose_timeout_sec) {
      critical = true;
      reason = "Localization pose update timed out";
    } else if (motion_sample_fault_) {
      critical = true;
      reason = motion_fault_reason_;
    } else if (pose_sample_fault_) {
      critical = true;
      reason = pose_fault_reason_;
    } else if (!have_correction_) {
      critical = true;
      reason = "Waiting for map_to_odom updates";
    } else if (!std::isfinite(correction_age) || correction_age < 0.0 ||
      correction_age > config_.correction_timeout_sec)
    {
      critical = true;
      reason = "map_to_odom update timed out";
    } else if (correction_sample_fault_) {
      critical = true;
      reason = correction_fault_reason_;
    }

    double weighted_score = 0.0;
    double available_weight = 0.0;
    if (have_odometry_ && have_pose_ && std::isfinite(update_age)) {
      weighted_score += config_.update_weight * freshnessScore(
        update_age, config_.pose_timeout_sec);
      available_weight += config_.update_weight;
    }
    if (motion_sample_valid_) {
      const double linear_ratio = linear_velocity_ / config_.max_linear_velocity_mps;
      const double angular_ratio = angular_velocity_ /
        (config_.max_angular_velocity_dps * kPi / 180.0);
      const double ratio = std::max(linear_ratio, angular_ratio);
      const double motion_score = ratio <= 0.8 ? 1.0 : clamp01((1.0 - ratio) / 0.2);
      weighted_score += config_.motion_weight * motion_score;
      available_weight += config_.motion_weight;
    }
    if (have_registration_ && registration_.valid &&
      std::isfinite(registration_age) && registration_age >= 0.0 &&
      registration_age <= config_.registration_timeout_sec)
    {
      const double fitness_score = clamp01(
        (registration_.fitness - config_.min_registration_fitness) /
        (1.0 - config_.min_registration_fitness));
      const double residual_score = clamp01(
        1.0 - registration_.mean_residual / config_.max_registration_residual_m);
      double registration_score = 0.7 * fitness_score + 0.3 * residual_score;
      if (!registration_.accepted) {
        registration_score = 0.0;
        critical = true;
      } else if (registration_.mean_residual > config_.max_registration_residual_m) {
        critical = true;
      }
      weighted_score += config_.registration_weight * registration_score;
      available_weight += config_.registration_weight;
      if (!registration_.accepted) {
        reason = "Scan-to-map registration was rejected";
      } else if (registration_.mean_residual > config_.max_registration_residual_m) {
        reason = "Scan-to-map registration residual exceeded the threshold";
      }
    } else if (have_registration_ &&
      (!std::isfinite(registration_age) || registration_age < 0.0 ||
      registration_age > config_.registration_timeout_sec))
    {
      critical = true;
      reason = "Registration quality update timed out";
    }

    double confidence = available_weight > 0.0 ? weighted_score / available_weight : 0.0;
    if (critical) {
      confidence = 0.0;
    }
    confidence = clamp01(confidence);
    status_.confidence = static_cast<float>(confidence);

    const bool good = !critical && confidence >= config_.localized_threshold;
    const bool critically_bad = critical || confidence < config_.degraded_threshold;
    if (good) {
      ++consecutive_good_frames_;
      consecutive_bad_frames_ = 0U;
      consecutive_critical_frames_ = 0U;
    } else {
      consecutive_good_frames_ = 0U;
      ++consecutive_bad_frames_;
      if (critically_bad) {
        ++consecutive_critical_frames_;
      } else {
        consecutive_critical_frames_ = 0U;
        if (reason == "Localization quality is healthy") {
          reason = "Confidence is below the localized threshold";
        }
      }
    }

    if (consecutive_critical_frames_ >= config_.bad_frames_to_lost) {
      transitionLocked(LocalizationQualityState::LOST, reason);
    } else if (state_ == LocalizationQualityState::UNKNOWN &&
      consecutive_bad_frames_ >= config_.bad_frames_to_degraded)
    {
      transitionLocked(LocalizationQualityState::DEGRADED, reason);
    } else if (state_ == LocalizationQualityState::LOCALIZED &&
      consecutive_bad_frames_ >= config_.bad_frames_to_degraded)
    {
      transitionLocked(LocalizationQualityState::DEGRADED, reason);
    } else if ((state_ == LocalizationQualityState::UNKNOWN ||
      state_ == LocalizationQualityState::DEGRADED ||
      state_ == LocalizationQualityState::LOST) &&
      consecutive_good_frames_ >= config_.good_frames_to_recover)
    {
      transitionLocked(LocalizationQualityState::LOCALIZED, "Quality remained healthy");
      reason = "Localization quality is healthy";
    }

    status_.pose_valid =
      state_ != LocalizationQualityState::UNKNOWN &&
      state_ != LocalizationQualityState::LOST && !critical && !motion_sample_fault_ &&
      !pose_sample_fault_ && !correction_sample_fault_;
    status_.reason = reason;
  }

  status_.state = toString(state_);
  status_.consecutive_good_frames = saturatingCount(consecutive_good_frames_);
  status_.consecutive_bad_frames = saturatingCount(consecutive_bad_frames_);
  const auto result = status_;
  if (publish) {
    status_publisher_->publish(result);
  }
  return result;
}

double LocalizationQualityMonitor::yawFromQuaternion(
  const geometry_msgs::msg::Quaternion & orientation)
{
  return std::atan2(
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
    1.0 - 2.0 *
    (orientation.y * orientation.y + orientation.z * orientation.z));
}

double LocalizationQualityMonitor::wrappedAngleDifference(
  const double current, const double previous)
{
  const double difference = current - previous;
  return std::atan2(std::sin(difference), std::cos(difference));
}

bool LocalizationQualityMonitor::finitePose(const geometry_msgs::msg::Pose & pose)
{
  const auto & position = pose.position;
  const auto & orientation = pose.orientation;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
    !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
    !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
    !std::isfinite(orientation.w))
  {
    return false;
  }
  const double norm_squared =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  return norm_squared > 1.0e-12;
}

double LocalizationQualityMonitor::ageSeconds(
  const rclcpp::Time & now, const rclcpp::Time & then)
{
  return (now - then).seconds();
}

float LocalizationQualityMonitor::unavailable()
{
  return std::numeric_limits<float>::quiet_NaN();
}

void LocalizationQualityMonitor::loadConfig(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & exception) {
    throw std::runtime_error(
            "Unable to load localization quality config '" + path + "': " + exception.what());
  }
  const auto quality = root["localization_quality"];
  if (!quality || !quality.IsMap()) {
    throw std::runtime_error("Missing localization_quality configuration section");
  }
  config_.publish_hz = requiredValue<double>(quality, "publish_hz");
  config_.pose_timeout_sec = requiredValue<double>(quality, "pose_timeout_sec");
  config_.correction_timeout_sec = requiredValue<double>(quality, "correction_timeout_sec");
  config_.registration_timeout_sec = requiredValue<double>(quality, "registration_timeout_sec");
  config_.max_position_jump_m = requiredValue<double>(quality, "max_position_jump_m");
  config_.max_yaw_jump_deg = requiredValue<double>(quality, "max_yaw_jump_deg");
  config_.max_linear_velocity_mps = requiredValue<double>(quality, "max_linear_velocity_mps");
  config_.max_angular_velocity_dps = requiredValue<double>(quality, "max_angular_velocity_dps");
  config_.max_correction_position_jump_m =
    requiredValue<double>(quality, "max_correction_position_jump_m");
  config_.max_correction_yaw_jump_deg =
    requiredValue<double>(quality, "max_correction_yaw_jump_deg");
  config_.min_registration_fitness = requiredValue<double>(quality, "min_registration_fitness");
  config_.max_registration_residual_m =
    requiredValue<double>(quality, "max_registration_residual_m");
  config_.bad_frames_to_degraded = requiredValue<std::size_t>(quality, "bad_frames_to_degraded");
  config_.bad_frames_to_lost = requiredValue<std::size_t>(quality, "bad_frames_to_lost");
  config_.good_frames_to_recover = requiredValue<std::size_t>(quality, "good_frames_to_recover");
  const auto confidence = quality["confidence"];
  const auto weights = quality["weights"];
  if (!confidence || !confidence.IsMap() || !weights || !weights.IsMap()) {
    throw std::runtime_error("Localization quality confidence and weights must be maps");
  }
  config_.localized_threshold = requiredValue<double>(confidence, "localized_threshold");
  config_.degraded_threshold = requiredValue<double>(confidence, "degraded_threshold");
  config_.update_weight = requiredValue<double>(weights, "update");
  config_.motion_weight = requiredValue<double>(weights, "motion");
  config_.registration_weight = requiredValue<double>(weights, "registration");
  config_.overlap_weight = requiredValue<double>(weights, "overlap");

  if (config_.publish_hz <= 0.0 || config_.pose_timeout_sec <= 0.0 ||
    config_.correction_timeout_sec <= 0.0 || config_.registration_timeout_sec <= 0.0 ||
    config_.max_position_jump_m <= 0.0 || config_.max_yaw_jump_deg <= 0.0 ||
    config_.max_linear_velocity_mps <= 0.0 || config_.max_angular_velocity_dps <= 0.0 ||
    config_.max_correction_position_jump_m <= 0.0 ||
    config_.max_correction_yaw_jump_deg <= 0.0 ||
    config_.max_registration_residual_m <= 0.0)
  {
    throw std::runtime_error("Localization quality rates, timeouts, and limits must be positive");
  }
  if (config_.min_registration_fitness < 0.0 || config_.min_registration_fitness >= 1.0 ||
    config_.degraded_threshold < 0.0 ||
    config_.localized_threshold > 1.0 ||
    config_.degraded_threshold >= config_.localized_threshold)
  {
    throw std::runtime_error("Localization quality confidence and fitness thresholds are invalid");
  }
  if (config_.bad_frames_to_degraded == 0U || config_.bad_frames_to_lost == 0U ||
    config_.good_frames_to_recover == 0U ||
    config_.bad_frames_to_lost < config_.bad_frames_to_degraded)
  {
    throw std::runtime_error("Localization quality hysteresis frame counts are invalid");
  }
  if (config_.update_weight < 0.0 || config_.motion_weight < 0.0 ||
    config_.registration_weight < 0.0 || config_.overlap_weight < 0.0 ||
    config_.update_weight + config_.motion_weight + config_.registration_weight +
    config_.overlap_weight <= 0.0)
  {
    throw std::runtime_error("Localization quality weights must be non-negative with a positive sum");
  }
}

void LocalizationQualityMonitor::timerCallback()
{
  const auto context = context_provider_ ? context_provider_() : LocalizationQualityContext{};
  evaluate(node_.now(), context, true);
}

void LocalizationQualityMonitor::transitionLocked(
  const LocalizationQualityState next, const std::string & reason)
{
  if (next == state_) {
    return;
  }
  const auto previous = state_;
  state_ = next;
  if (next == LocalizationQualityState::LOST) {
    RCLCPP_ERROR(
      node_.get_logger(), "[LOCALIZATION_QUALITY] %s -> %s: %s",
      toString(previous), toString(next), reason.c_str());
  } else if (next == LocalizationQualityState::DEGRADED) {
    RCLCPP_WARN(
      node_.get_logger(), "[LOCALIZATION_QUALITY] %s -> %s: %s",
      toString(previous), toString(next), reason.c_str());
  } else {
    RCLCPP_INFO(
      node_.get_logger(), "[LOCALIZATION_QUALITY] %s -> %s: %s",
      toString(previous), toString(next), reason.c_str());
  }
  if (state_change_callback_) {
    state_change_callback_(previous, next, reason);
  }
}

}  // namespace slam_system_manager
