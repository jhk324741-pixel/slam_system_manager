#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include <unistd.h>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "slam_system_manager/relocalization_adapter.hpp"

namespace slam_system_manager
{
namespace fs = std::filesystem;
using namespace std::chrono_literals;

class RelocalizationAdapterTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    root_ = fs::temp_directory_path() /
      ("slam_system_manager_relocalization_test_" + std::to_string(getpid()));
    fs::remove_all(root_);
    fs::create_directories(root_ / "test_map");
  }

  void TearDown() override
  {
    fs::remove_all(root_);
  }

  SystemConfig config() const
  {
    SystemConfig config;
    config.map_root = root_.string();
    config.frames.map = "map";
    config.frames.odom = "odom";
    config.frames.base = "base_link";
    config.frames.lidar = "lidar";
    config.topics.initial_pose = "/test/initialpose";
    config.topics.localization_raw_odometry = "/test/raw_odometry";
    config.topics.localization_correction = "/test/map_to_odom";
    config.topics.localization_odometry = "/test/localization/odometry";
    config.topics.localization_pose = "/test/localization/pose";
    config.last_pose.enable = true;
    config.last_pose.save_interval_sec = 0.1;
    config.relocalization.initial_pose_wait_timeout_sec = 1.0;
    config.relocalization.min_initial_pose_subscribers = 2U;
    return config;
  }

  fs::path root_;
};

TEST_F(RelocalizationAdapterTest, ValidatesFrameAndNormalizesQuaternion)
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.frame_id = "map";
  pose.pose.pose.orientation.w = 2.0;
  std::string error;
  ASSERT_TRUE(RelocalizationAdapter::validatePose(&pose, "map", &error)) << error;
  EXPECT_DOUBLE_EQ(pose.pose.pose.orientation.w, 1.0);

  pose.header.frame_id = "odom";
  EXPECT_FALSE(RelocalizationAdapter::validatePose(&pose, "map", &error));
  EXPECT_NE(error.find("frame must be 'map'"), std::string::npos);

  pose.header.frame_id = "map";
  pose.pose.pose.orientation.w = 0.0;
  EXPECT_FALSE(RelocalizationAdapter::validatePose(&pose, "map", &error));
  EXPECT_NE(error.find("zero length"), std::string::npos);
}

TEST_F(RelocalizationAdapterTest, RejectsMissingAndUnsafeLastPose)
{
  auto node = std::make_shared<rclcpp::Node>("relocalization_load_test");
  RelocalizationAdapter adapter(*node, config(), nullptr, nullptr);
  std::string error;
  EXPECT_FALSE(adapter.loadLastPose(root_ / "test_map", &error));
  EXPECT_NE(error.find("not found"), std::string::npos);
  EXPECT_FALSE(adapter.loadLastPose(root_.parent_path(), &error));
  EXPECT_NE(error.find("escapes"), std::string::npos);
}

TEST_F(RelocalizationAdapterTest, FusesPoseAndWritesLastPoseAtomically)
{
  auto node = std::make_shared<rclcpp::Node>("relocalization_fusion_test");
  bool confirmed = false;
  RelocalizationAdapter adapter(
    *node, config(), nullptr, [&confirmed]() {confirmed = true;});
  auto correction_publisher = node->create_publisher<nav_msgs::msg::Odometry>(
    "/test/map_to_odom", 10);
  auto raw_publisher = node->create_publisher<nav_msgs::msg::Odometry>(
    "/test/raw_odometry", 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  adapter.resetSession();
  std::this_thread::sleep_for(5ms);
  nav_msgs::msg::Odometry correction;
  correction.header.stamp = node->now();
  correction.header.frame_id = "map";
  correction.pose.pose.position.x = 10.0;
  correction.pose.pose.orientation.w = 1.0;
  correction_publisher->publish(correction);
  for (int index = 0; index < 20 && !confirmed; ++index) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(confirmed);
  adapter.setLocalized(true);

  nav_msgs::msg::Odometry raw;
  raw.header.stamp = node->now();
  raw.header.frame_id = "odom";
  raw.pose.pose.position.x = 2.0;
  raw.pose.pose.orientation.w = 1.0;
  raw_publisher->publish(raw);
  for (int index = 0; index < 20; ++index) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  std::string error;
  ASSERT_TRUE(adapter.requestLastPoseSave(root_ / "test_map", &error)) << error;
  const auto last_pose_path = root_ / "test_map" / "last_pose.yaml";
  for (int index = 0; index < 100 && !fs::is_regular_file(last_pose_path); ++index) {
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(fs::is_regular_file(last_pose_path));

  const auto loaded = adapter.loadLastPose(root_ / "test_map", &error);
  ASSERT_TRUE(loaded) << error;
  EXPECT_EQ(loaded->header.frame_id, "map");
  EXPECT_NEAR(loaded->pose.pose.position.x, 12.0, 1.0e-6);
  EXPECT_DOUBLE_EQ(loaded->pose.pose.orientation.w, 1.0);
}

}  // namespace slam_system_manager
