#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "slam_system_manager/localization_quality_monitor.hpp"

namespace slam_system_manager
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

class LocalizationQualityMonitorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
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
    static std::atomic<unsigned int> sequence{0U};
    node_ = std::make_shared<rclcpp::Node>(
      "localization_quality_test_" + std::to_string(sequence.fetch_add(1U)));
    monitor_ = std::make_unique<LocalizationQualityMonitor>(
      *node_, LOCALIZATION_QUALITY_TEST_CONFIG,
      "/quality_test/odometry", "/quality_test/pose", "/quality_test/correction",
      "map",
      []() {return healthyContext();});
  }

  static LocalizationQualityContext healthyContext()
  {
    LocalizationQualityContext context;
    context.localization_active = true;
    context.localization_initialized = true;
    context.localization_process_alive = true;
    context.sensors_healthy = true;
    return context;
  }

  static rclcpp::Time at(const double seconds)
  {
    return rclcpp::Time(static_cast<std::int64_t>(seconds * 1.0e9), RCL_ROS_TIME);
  }

  static geometry_msgs::msg::Quaternion yawQuaternion(const double yaw)
  {
    geometry_msgs::msg::Quaternion orientation;
    orientation.z = std::sin(yaw * 0.5);
    orientation.w = std::cos(yaw * 0.5);
    return orientation;
  }

  static nav_msgs::msg::Odometry odometry(
    const double seconds, const double x = 0.0, const double yaw = 0.0)
  {
    nav_msgs::msg::Odometry message;
    message.header.stamp = at(seconds);
    message.header.frame_id = "map";
    message.child_frame_id = "base_link";
    message.pose.pose.position.x = x;
    message.pose.pose.orientation = yawQuaternion(yaw);
    return message;
  }

  static geometry_msgs::msg::PoseWithCovarianceStamped pose(
    const double seconds, const double x = 0.0, const double yaw = 0.0)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.stamp = at(seconds);
    message.header.frame_id = "map";
    message.pose.pose.position.x = x;
    message.pose.pose.orientation = yawQuaternion(yaw);
    return message;
  }

  static msg::RegistrationQuality registration(
    const double seconds, const bool accepted = true)
  {
    msg::RegistrationQuality message;
    message.header.stamp = at(seconds);
    message.header.frame_id = "map";
    message.valid = true;
    message.accepted = accepted;
    message.fitness = accepted ? 0.98F : 0.50F;
    message.mean_residual = accepted ? 0.20F : 0.80F;
    message.correspondence_count = accepted ? 980U : 500U;
    message.source_point_count = 1000U;
    return message;
  }

  void seedQuality(const double seconds = 1.0)
  {
    monitor_->observeCorrection(odometry(seconds), at(seconds));
    monitor_->observeRegistration(registration(seconds), at(seconds));
  }

  msg::LocalizationStatus goodFrame(
    const double seconds, const double x = 0.0, const double yaw = 0.0)
  {
    const auto odom = odometry(seconds, x, yaw);
    const auto output_pose = pose(seconds, x, yaw);
    monitor_->observeOdometry(odom, at(seconds));
    monitor_->observePose(output_pose, at(seconds));
    return monitor_->evaluate(at(seconds), healthyContext());
  }

  void reachLocalized()
  {
    seedQuality();
    goodFrame(1.0, 0.00);
    goodFrame(1.1, 0.01);
    const auto status = goodFrame(1.2, 0.02);
    ASSERT_EQ(status.state, "LOCALIZED");
    ASSERT_TRUE(status.pose_valid);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<LocalizationQualityMonitor> monitor_;
};

TEST_F(LocalizationQualityMonitorTest, NormalContinuousPosesBecomeLocalized)
{
  reachLocalized();
  const auto status = goodFrame(1.3, 0.03);
  EXPECT_EQ(status.state, "LOCALIZED");
  EXPECT_TRUE(status.pose_valid);
  EXPECT_GE(status.confidence, 0.75F);
}

TEST_F(LocalizationQualityMonitorTest, SingleBadFrameDoesNotImmediatelyBecomeLost)
{
  reachLocalized();
  auto jump = odometry(1.3, 2.0);
  monitor_->observeOdometry(jump, at(1.3));
  monitor_->observePose(pose(1.3, 2.0), at(1.3));
  const auto status = monitor_->evaluate(at(1.3), healthyContext());
  EXPECT_EQ(status.state, "LOCALIZED");
  EXPECT_FALSE(status.pose_valid);
  EXPECT_EQ(status.consecutive_bad_frames, 1U);
}

TEST_F(LocalizationQualityMonitorTest, ConsecutiveBadFramesBecomeDegradedThenLost)
{
  reachLocalized();
  auto invalid_pose = pose(1.3);
  invalid_pose.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  monitor_->observePose(invalid_pose, at(1.3));
  EXPECT_EQ(monitor_->evaluate(at(1.3), healthyContext()).state, "LOCALIZED");
  EXPECT_EQ(monitor_->evaluate(at(1.4), healthyContext()).state, "DEGRADED");
  EXPECT_EQ(monitor_->evaluate(at(1.5), healthyContext()).state, "DEGRADED");
  const auto lost = monitor_->evaluate(at(1.6), healthyContext());
  EXPECT_EQ(lost.state, "LOST");
  EXPECT_FALSE(lost.pose_valid);
}

TEST_F(LocalizationQualityMonitorTest, LostRequiresConsecutiveGoodFramesToRecover)
{
  reachLocalized();
  auto invalid_pose = pose(1.3);
  invalid_pose.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  monitor_->observePose(invalid_pose, at(1.3));
  for (int index = 0; index < 4; ++index) {
    monitor_->evaluate(at(1.3 + index * 0.1), healthyContext());
  }
  ASSERT_EQ(monitor_->status().state, "LOST");

  EXPECT_EQ(goodFrame(1.7, 0.03).state, "LOST");
  EXPECT_EQ(goodFrame(1.8, 0.04).state, "LOST");
  const auto recovered = goodFrame(1.9, 0.05);
  EXPECT_EQ(recovered.state, "LOCALIZED");
  EXPECT_TRUE(recovered.pose_valid);
}

TEST_F(LocalizationQualityMonitorTest, PositionJumpAndNonPhysicalVelocityAreDetected)
{
  reachLocalized();
  monitor_->observeOdometry(odometry(1.3, 1.0), at(1.3));
  monitor_->observePose(pose(1.3, 1.0), at(1.3));
  const auto status = monitor_->evaluate(at(1.3), healthyContext());
  EXPECT_GT(status.position_jump, 0.5F);
  EXPECT_GT(status.linear_velocity, 2.0F);
  EXPECT_NE(status.reason.find("Position jump"), std::string::npos);
}

TEST_F(LocalizationQualityMonitorTest, YawWrapAroundUsesShortestAngle)
{
  seedQuality();
  goodFrame(1.0, 0.0, 179.0 * kPi / 180.0);
  const auto status = goodFrame(1.1, 0.0, -179.0 * kPi / 180.0);
  EXPECT_NEAR(status.yaw_jump, 2.0 * kPi / 180.0, 1.0e-3);
  EXPECT_LT(status.angular_velocity, 100.0 * kPi / 180.0);
}

TEST_F(LocalizationQualityMonitorTest, PoseTimeoutBecomesLostWhileProcessRemainsAlive)
{
  reachLocalized();
  auto context = healthyContext();
  context.localization_process_alive = true;
  EXPECT_EQ(monitor_->evaluate(at(1.8), context).state, "LOCALIZED");
  EXPECT_EQ(monitor_->evaluate(at(1.9), context).state, "DEGRADED");
  monitor_->evaluate(at(2.0), context);
  const auto lost = monitor_->evaluate(at(2.1), context);
  EXPECT_EQ(lost.state, "LOST");
  EXPECT_NE(lost.reason.find("timed out"), std::string::npos);
}

TEST_F(LocalizationQualityMonitorTest, NaNInputIsRejectedWithoutRefreshingPose)
{
  reachLocalized();
  auto invalid = odometry(1.3);
  invalid.pose.pose.position.y = std::numeric_limits<double>::infinity();
  monitor_->observeOdometry(invalid, at(1.3));
  const auto status = monitor_->evaluate(at(1.3), healthyContext());
  EXPECT_FALSE(status.pose_valid);
  EXPECT_NE(status.reason.find("NaN"), std::string::npos);
}

TEST_F(LocalizationQualityMonitorTest, TimestampRegressionIsDetected)
{
  reachLocalized();
  monitor_->observeOdometry(odometry(1.1, 0.03), at(1.3));
  const auto status = monitor_->evaluate(at(1.3), healthyContext());
  EXPECT_FALSE(status.pose_valid);
  EXPECT_NE(status.reason.find("timestamp"), std::string::npos);
}

TEST_F(LocalizationQualityMonitorTest, RejectedRegistrationDrivesLostWithHysteresis)
{
  reachLocalized();
  monitor_->observeRegistration(registration(1.3, false), at(1.3));
  for (int index = 0; index < 3; ++index) {
    monitor_->evaluate(at(1.3 + index * 0.1), healthyContext());
  }
  const auto lost = monitor_->evaluate(at(1.6), healthyContext());
  EXPECT_EQ(lost.state, "LOST");
  EXPECT_NE(lost.reason.find("rejected"), std::string::npos);
}

TEST_F(LocalizationQualityMonitorTest, ExcessiveRegistrationResidualIsDetected)
{
  reachLocalized();
  auto poor_registration = registration(1.3);
  poor_registration.mean_residual = 0.9F;
  monitor_->observeRegistration(poor_registration, at(1.3));
  const auto status = monitor_->evaluate(at(1.3), healthyContext());
  EXPECT_FALSE(status.pose_valid);
  EXPECT_NE(status.reason.find("residual"), std::string::npos);
}

TEST_F(LocalizationQualityMonitorTest, CorrectionJumpIsDetected)
{
  reachLocalized();
  monitor_->observeCorrection(odometry(1.3, 2.0), at(1.3));
  const auto status = monitor_->evaluate(at(1.3), healthyContext());
  EXPECT_FALSE(status.pose_valid);
  EXPECT_NE(status.reason.find("map_to_odom position"), std::string::npos);
}

TEST_F(LocalizationQualityMonitorTest, IndependentFaultReasonSurvivesOtherHealthyInputs)
{
  reachLocalized();
  monitor_->observeCorrection(odometry(1.3, 2.0), at(1.3));
  monitor_->observeOdometry(odometry(1.3, 0.03), at(1.3));
  monitor_->observePose(pose(1.3, 0.03), at(1.3));
  const auto status = monitor_->evaluate(at(1.3), healthyContext());
  EXPECT_FALSE(status.pose_valid);
  EXPECT_NE(status.reason.find("map_to_odom position"), std::string::npos);
}

TEST_F(LocalizationQualityMonitorTest, MissingOverlapDoesNotReduceConfidenceToZero)
{
  reachLocalized();
  const auto status = goodFrame(1.3, 0.03);
  EXPECT_TRUE(std::isnan(status.map_overlap_ratio));
  EXPECT_GE(status.confidence, 0.75F);
}

}  // namespace
}  // namespace slam_system_manager
