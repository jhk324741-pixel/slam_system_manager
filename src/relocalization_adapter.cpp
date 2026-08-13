#include "slam_system_manager/relocalization_adapter.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <system_error>

#include <unistd.h>

#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
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
    throw std::runtime_error("Missing Last Pose key: " + key);
  }
  return value.as<T>();
}

bool finitePosition(const geometry_msgs::msg::Point & position)
{
  return std::isfinite(position.x) && std::isfinite(position.y) &&
         std::isfinite(position.z);
}

}  // namespace

RelocalizationAdapter::RelocalizationAdapter(
  rclcpp::Node & node, const SystemConfig & config,
  InitialPoseCallback initial_pose_callback,
  LocalizationConfirmedCallback localization_confirmed_callback)
: node_(node),
  config_(config),
  initial_pose_callback_(std::move(initial_pose_callback)),
  localization_confirmed_callback_(std::move(localization_confirmed_callback))
{
  initial_pose_publisher_ =
    node_.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    config_.topics.initial_pose, rclcpp::QoS(10).reliable());
  localization_pose_publisher_ =
    node_.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    config_.topics.localization_pose, 10);
  localization_odometry_publisher_ =
    node_.create_publisher<nav_msgs::msg::Odometry>(config_.topics.localization_odometry, 10);

  callback_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  initial_pose_subscription_ =
    node_.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    config_.topics.initial_pose, rclcpp::QoS(10).reliable(),
    std::bind(&RelocalizationAdapter::handleInitialPose, this, std::placeholders::_1),
    options);
  raw_odometry_subscription_ = node_.create_subscription<nav_msgs::msg::Odometry>(
    config_.topics.localization_raw_odometry, 10,
    std::bind(&RelocalizationAdapter::handleRawOdometry, this, std::placeholders::_1),
    options);
  correction_subscription_ = node_.create_subscription<nav_msgs::msg::Odometry>(
    config_.topics.localization_correction, 10,
    std::bind(&RelocalizationAdapter::handleCorrection, this, std::placeholders::_1),
    options);

  save_thread_ = std::thread(&RelocalizationAdapter::saveWorker, this);
}

RelocalizationAdapter::~RelocalizationAdapter()
{
  {
    std::lock_guard<std::mutex> lock(save_mutex_);
    stop_worker_ = true;
  }
  save_condition_.notify_one();
  if (save_thread_.joinable()) {
    save_thread_.join();
  }
}

void RelocalizationAdapter::resetSession()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  localized_ = false;
  correction_available_ = false;
  relocalization_started_at_ = node_.now();
  correction_ = nav_msgs::msg::Odometry{};
  latest_trusted_pose_.reset();
}

void RelocalizationAdapter::setLocalized(const bool localized)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  localized_ = localized;
  if (!localized_) {
    latest_trusted_pose_.reset();
  }
}

bool RelocalizationAdapter::localizationConfirmed() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return correction_available_;
}

bool RelocalizationAdapter::initialPoseConsumerReady() const
{
  return initial_pose_publisher_->get_subscription_count() >=
         config_.relocalization.min_initial_pose_subscribers;
}

bool RelocalizationAdapter::waitForInitialPoseConsumer(const double timeout_sec) const
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(timeout_sec));
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    if (initialPoseConsumerReady()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return initialPoseConsumerReady();
}

bool RelocalizationAdapter::publishInitialPose(
  geometry_msgs::msg::PoseWithCovarianceStamped pose, std::string * error)
{
  if (!validatePose(&pose, config_.frames.map, error)) {
    return false;
  }
  if (!initialPoseConsumerReady()) {
    setError(error, "Localization initial pose subscriber is not ready");
    return false;
  }
  pose.header.stamp = node_.now();
  initial_pose_publisher_->publish(pose);
  return true;
}

std::optional<geometry_msgs::msg::PoseWithCovarianceStamped>
RelocalizationAdapter::loadLastPose(
  const fs::path & map_directory, std::string * error) const
{
  const auto normalized_directory = map_directory.lexically_normal();
  const auto normalized_root = fs::path(config_.map_root).lexically_normal();
  if (!normalized_directory.is_absolute() ||
    normalized_directory.parent_path() != normalized_root)
  {
    setError(error, "Last Pose path escapes the configured map root");
    return std::nullopt;
  }

  const auto path = normalized_directory / "last_pose.yaml";
  if (!fs::is_regular_file(path)) {
    setError(error, "Last Pose not found: " + path.string());
    return std::nullopt;
  }

  try {
    const auto root = YAML::LoadFile(path.string());
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id = requiredValue<std::string>(root, "frame_id");
    const auto position = root["position"];
    const auto orientation = root["orientation"];
    if (!position || !position.IsMap() || !orientation || !orientation.IsMap()) {
      throw std::runtime_error("Last Pose position and orientation must be maps");
    }
    pose.pose.pose.position.x = requiredValue<double>(position, "x");
    pose.pose.pose.position.y = requiredValue<double>(position, "y");
    pose.pose.pose.position.z = requiredValue<double>(position, "z");
    pose.pose.pose.orientation.x = requiredValue<double>(orientation, "x");
    pose.pose.pose.orientation.y = requiredValue<double>(orientation, "y");
    pose.pose.pose.orientation.z = requiredValue<double>(orientation, "z");
    pose.pose.pose.orientation.w = requiredValue<double>(orientation, "w");
    if (!validatePose(&pose, config_.frames.map, error)) {
      return std::nullopt;
    }
    pose.header.stamp = node_.now();
    return pose;
  } catch (const std::exception & exception) {
    setError(error, "Unable to load Last Pose '" + path.string() + "': " + exception.what());
    return std::nullopt;
  }
}

bool RelocalizationAdapter::requestLastPoseSave(
  const fs::path & map_directory, std::string * error)
{
  if (!config_.last_pose.enable) {
    setError(error, "Last Pose saving is disabled");
    return false;
  }

  const auto normalized_directory = map_directory.lexically_normal();
  const auto normalized_root = fs::path(config_.map_root).lexically_normal();
  if (!normalized_directory.is_absolute() ||
    normalized_directory.parent_path() != normalized_root ||
    !fs::is_directory(normalized_directory))
  {
    setError(error, "Last Pose map directory is invalid");
    return false;
  }

  SaveRequest request;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!localized_ || !latest_trusted_pose_) {
      setError(error, "No trusted localized pose is available");
      return false;
    }
    request.map_directory = normalized_directory;
    request.pose = *latest_trusted_pose_;
  }
  {
    std::lock_guard<std::mutex> lock(save_mutex_);
    pending_save_ = std::move(request);
  }
  save_condition_.notify_one();
  return true;
}

bool RelocalizationAdapter::validatePose(
  geometry_msgs::msg::PoseWithCovarianceStamped * pose,
  const std::string & expected_frame, std::string * error)
{
  if (pose == nullptr) {
    setError(error, "Initial pose is null");
    return false;
  }
  if (pose->header.frame_id.empty()) {
    pose->header.frame_id = expected_frame;
  }
  if (pose->header.frame_id != expected_frame) {
    setError(
      error, "Initial pose frame must be '" + expected_frame + "', got '" +
      pose->header.frame_id + "'");
    return false;
  }
  if (!finitePosition(pose->pose.pose.position)) {
    setError(error, "Initial pose position contains a non-finite value");
    return false;
  }

  auto & orientation = pose->pose.pose.orientation;
  if (!std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
    !std::isfinite(orientation.z) || !std::isfinite(orientation.w))
  {
    setError(error, "Initial pose orientation contains a non-finite value");
    return false;
  }
  const double norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  if (norm < 1.0e-6) {
    setError(error, "Initial pose quaternion has zero length");
    return false;
  }
  orientation.x /= norm;
  orientation.y /= norm;
  orientation.z /= norm;
  orientation.w /= norm;
  return true;
}

void RelocalizationAdapter::setError(std::string * output, const std::string & message)
{
  if (output != nullptr) {
    *output = message;
  }
}

void RelocalizationAdapter::writeLastPose(const SaveRequest & request)
{
  YAML::Emitter output;
  const auto & pose = request.pose.pose.pose;
  output << YAML::BeginMap;
  output << YAML::Key << "frame_id" << YAML::Value << request.pose.header.frame_id;
  output << YAML::Key << "position" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "x" << YAML::Value << pose.position.x;
  output << YAML::Key << "y" << YAML::Value << pose.position.y;
  output << YAML::Key << "z" << YAML::Value << pose.position.z;
  output << YAML::EndMap;
  output << YAML::Key << "orientation" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "x" << YAML::Value << pose.orientation.x;
  output << YAML::Key << "y" << YAML::Value << pose.orientation.y;
  output << YAML::Key << "z" << YAML::Value << pose.orientation.z;
  output << YAML::Key << "w" << YAML::Value << pose.orientation.w;
  output << YAML::EndMap;
  output << YAML::EndMap;
  if (!output.good()) {
    throw std::runtime_error("Unable to serialize Last Pose");
  }

  const auto path = request.map_directory / "last_pose.yaml";
  const auto temporary = request.map_directory /
    (".last_pose.yaml.tmp." + std::to_string(getpid()));
  {
    std::ofstream stream(temporary, std::ios::out | std::ios::trunc);
    if (!stream) {
      throw std::runtime_error("Unable to open temporary Last Pose file: " + temporary.string());
    }
    stream << output.c_str() << '\n';
    stream.flush();
    if (!stream) {
      throw std::runtime_error("Unable to write Last Pose file: " + temporary.string());
    }
  }
  std::error_code filesystem_error;
  fs::rename(temporary, path, filesystem_error);
  if (filesystem_error) {
    fs::remove(temporary);
    throw std::runtime_error("Unable to install Last Pose file: " + filesystem_error.message());
  }
}

void RelocalizationAdapter::handleInitialPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
{
  auto pose = *message;
  std::string error;
  if (!validatePose(&pose, config_.frames.map, &error)) {
    RCLCPP_ERROR(node_.get_logger(), "[LOCALIZATION] Ignoring invalid initial pose: %s", error.c_str());
    return;
  }
  if (initial_pose_callback_) {
    initial_pose_callback_(pose);
  }
}

void RelocalizationAdapter::handleRawOdometry(
  const nav_msgs::msg::Odometry::SharedPtr message)
{
  nav_msgs::msg::Odometry correction;
  bool localized = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!correction_available_) {
      return;
    }
    correction = correction_;
    localized = localized_;
  }

  tf2::Transform map_to_odom;
  tf2::Transform odom_to_base;
  tf2::fromMsg(correction.pose.pose, map_to_odom);
  tf2::fromMsg(message->pose.pose, odom_to_base);
  const auto map_to_base = map_to_odom * odom_to_base;

  nav_msgs::msg::Odometry output = *message;
  output.header.frame_id = config_.frames.map;
  output.child_frame_id = config_.frames.base;
  output.pose.pose.position.x = map_to_base.getOrigin().x();
  output.pose.pose.position.y = map_to_base.getOrigin().y();
  output.pose.pose.position.z = map_to_base.getOrigin().z();
  output.pose.pose.orientation = tf2::toMsg(map_to_base.getRotation());
  localization_odometry_publisher_->publish(output);

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header = output.header;
  pose.pose = output.pose;
  localization_pose_publisher_->publish(pose);

  if (localized) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (localized_) {
      latest_trusted_pose_ = pose;
    }
  }
}

void RelocalizationAdapter::handleCorrection(
  const nav_msgs::msg::Odometry::SharedPtr message)
{
  geometry_msgs::msg::PoseWithCovarianceStamped correction_pose;
  correction_pose.header = message->header;
  correction_pose.pose = message->pose;
  std::string error;
  if (!validatePose(&correction_pose, config_.frames.map, &error)) {
    RCLCPP_ERROR(
      node_.get_logger(), "[LOCALIZATION] Ignoring invalid correction: %s", error.c_str());
    return;
  }

  bool first_confirmation = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (rclcpp::Time(message->header.stamp) <= relocalization_started_at_) {
      return;
    }
    first_confirmation = !correction_available_;
    correction_ = *message;
    correction_.pose = correction_pose.pose;
    correction_available_ = true;
  }
  if (first_confirmation && localization_confirmed_callback_) {
    localization_confirmed_callback_();
  }
}

void RelocalizationAdapter::saveWorker()
{
  while (true) {
    std::optional<SaveRequest> request;
    {
      std::unique_lock<std::mutex> lock(save_mutex_);
      save_condition_.wait(lock, [this]() {return stop_worker_ || pending_save_.has_value();});
      if (!pending_save_ && stop_worker_) {
        return;
      }
      request = std::move(pending_save_);
      pending_save_.reset();
    }

    try {
      writeLastPose(*request);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        node_.get_logger(), "[LOCALIZATION] Last Pose save failed: %s", exception.what());
    }
  }
}

}  // namespace slam_system_manager
