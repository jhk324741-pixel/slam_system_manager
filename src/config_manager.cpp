#include "slam_system_manager/config_manager.hpp"

#include <stdexcept>
#include <string>

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
    throw std::runtime_error("Missing required configuration key: " + key);
  }
  try {
    return node.as<T>();
  } catch (const YAML::Exception & exception) {
    throw std::runtime_error(
            "Invalid value for configuration key '" + key + "': " + exception.what());
  }
}

std::array<double, 3> requiredVector3(
  const YAML::Node & parent, const std::string & key)
{
  const auto node = parent[key];
  if (!node || !node.IsSequence() || node.size() != 3U) {
    throw std::runtime_error(
            "Configuration key '" + key + "' must contain exactly three numbers");
  }
  return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
}

void requireNonEmpty(const std::string & value, const std::string & key)
{
  if (value.empty()) {
    throw std::runtime_error("Configuration key '" + key + "' must not be empty");
  }
}

void requireAbsoluteTopic(const std::string & value, const std::string & key)
{
  requireNonEmpty(value, key);
  if (value.front() != '/') {
    throw std::runtime_error("Configuration topic '" + key + "' must start with '/'");
  }
}

}  // namespace

void ConfigManager::load(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & exception) {
    throw std::runtime_error("Unable to load system config '" + path + "': " + exception.what());
  }

  SystemConfig next;
  next.map_root = requiredValue<std::string>(root, "map_root");
  next.status_publish_hz = requiredValue<double>(root, "status_publish_hz");

  const auto frames = root["frames"];
  if (!frames || !frames.IsMap()) {
    throw std::runtime_error("Missing required configuration section: frames");
  }
  next.frames.map = requiredValue<std::string>(frames, "map");
  next.frames.odom = requiredValue<std::string>(frames, "odom");
  next.frames.base = requiredValue<std::string>(frames, "base");
  next.frames.lidar = requiredValue<std::string>(frames, "lidar");

  const auto topics = root["topics"];
  if (!topics || !topics.IsMap()) {
    throw std::runtime_error("Missing required configuration section: topics");
  }
  next.topics.pointcloud = requiredValue<std::string>(topics, "pointcloud");
  next.topics.imu = requiredValue<std::string>(topics, "imu");
  next.topics.initial_pose = requiredValue<std::string>(topics, "initial_pose");
  next.topics.localization_raw_odometry =
    requiredValue<std::string>(topics, "localization_raw_odometry");
  next.topics.localization_correction =
    requiredValue<std::string>(topics, "localization_correction");
  next.topics.localization_odometry =
    requiredValue<std::string>(topics, "localization_odometry");
  next.topics.localization_pose = requiredValue<std::string>(topics, "localization_pose");
  next.topics.system_status = requiredValue<std::string>(topics, "system_status");

  const auto last_pose = root["last_pose"];
  if (!last_pose || !last_pose.IsMap()) {
    throw std::runtime_error("Missing required configuration section: last_pose");
  }
  next.last_pose.enable = requiredValue<bool>(last_pose, "enable");
  next.last_pose.save_interval_sec = requiredValue<double>(last_pose, "save_interval_sec");

  const auto relocalization = root["relocalization"];
  if (!relocalization || !relocalization.IsMap()) {
    throw std::runtime_error("Missing required configuration section: relocalization");
  }
  next.relocalization.initial_pose_wait_timeout_sec =
    requiredValue<double>(relocalization, "initial_pose_wait_timeout_sec");
  next.relocalization.initial_pose_settle_sec =
    requiredValue<double>(relocalization, "initial_pose_settle_sec");
  next.relocalization.last_pose_retry_count =
    requiredValue<std::size_t>(relocalization, "last_pose_retry_count");
  next.relocalization.last_pose_retry_interval_sec =
    requiredValue<double>(relocalization, "last_pose_retry_interval_sec");
  next.relocalization.min_initial_pose_subscribers =
    requiredValue<std::size_t>(relocalization, "min_initial_pose_subscribers");

  const auto extrinsics = root["extrinsics"];
  const auto lidar_to_base = extrinsics ? extrinsics["lidar_to_base"] : YAML::Node{};
  if (!lidar_to_base || !lidar_to_base.IsMap()) {
    throw std::runtime_error("Missing required configuration section: extrinsics.lidar_to_base");
  }
  next.lidar_to_base.translation = requiredVector3(lidar_to_base, "translation");
  next.lidar_to_base.rotation_rpy = requiredVector3(lidar_to_base, "rotation_rpy");

  requireNonEmpty(next.map_root, "map_root");
  if (next.map_root.front() != '/') {
    throw std::runtime_error("Configuration key 'map_root' must be an absolute path");
  }
  requireNonEmpty(next.frames.map, "frames.map");
  requireNonEmpty(next.frames.odom, "frames.odom");
  requireNonEmpty(next.frames.base, "frames.base");
  requireNonEmpty(next.frames.lidar, "frames.lidar");
  requireAbsoluteTopic(next.topics.pointcloud, "topics.pointcloud");
  requireAbsoluteTopic(next.topics.imu, "topics.imu");
  requireAbsoluteTopic(next.topics.initial_pose, "topics.initial_pose");
  requireAbsoluteTopic(
    next.topics.localization_raw_odometry, "topics.localization_raw_odometry");
  requireAbsoluteTopic(
    next.topics.localization_correction, "topics.localization_correction");
  requireAbsoluteTopic(next.topics.localization_odometry, "topics.localization_odometry");
  requireAbsoluteTopic(next.topics.localization_pose, "topics.localization_pose");
  requireAbsoluteTopic(next.topics.system_status, "topics.system_status");

  if (next.status_publish_hz <= 0.0) {
    throw std::runtime_error("Configuration key 'status_publish_hz' must be greater than zero");
  }
  if (next.last_pose.save_interval_sec <= 0.0) {
    throw std::runtime_error(
            "Configuration key 'last_pose.save_interval_sec' must be greater than zero");
  }
  if (next.relocalization.initial_pose_wait_timeout_sec <= 0.0) {
    throw std::runtime_error(
            "Configuration key 'relocalization.initial_pose_wait_timeout_sec' must be greater than zero");
  }
  if (next.relocalization.initial_pose_settle_sec < 0.0) {
    throw std::runtime_error(
            "Configuration key 'relocalization.initial_pose_settle_sec' must not be negative");
  }
  if (next.relocalization.last_pose_retry_count == 0U ||
    next.relocalization.last_pose_retry_interval_sec <= 0.0)
  {
    throw std::runtime_error(
            "Relocalization Last Pose retry count and interval must be greater than zero");
  }
  if (next.relocalization.min_initial_pose_subscribers < 2U) {
    throw std::runtime_error(
            "Configuration key 'relocalization.min_initial_pose_subscribers' must be at least 2");
  }

  config_ = std::move(next);
  loaded_ = true;
}

const SystemConfig & ConfigManager::get() const
{
  if (!loaded_) {
    throw std::logic_error("System configuration has not been loaded");
  }
  return config_;
}

bool ConfigManager::isLoaded() const noexcept
{
  return loaded_;
}

}  // namespace slam_system_manager
