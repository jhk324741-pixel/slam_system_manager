#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "slam_system_manager/msg/system_status.hpp"
#include "slam_system_manager/srv/get_system_status.hpp"
#include "slam_system_manager/system_manager.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace slam_system_manager
{
namespace
{

using namespace std::chrono_literals;

class RosEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

const auto ros_environment =
  ::testing::AddGlobalTestEnvironment(new RosEnvironment());

class SystemManagerApiTest : public ::testing::Test
{
protected:
  void startManager(const std::string & process_config)
  {
    static std::atomic<unsigned int> sequence{0U};
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("config_file", API_SYSTEM_CONFIG),
      rclcpp::Parameter("process_config_file", process_config),
      rclcpp::Parameter("sensor_config_file", API_SENSOR_CONFIG)
    });
    manager_ = std::make_shared<SystemManager>(options);
    client_node_ = std::make_shared<rclcpp::Node>(
      "phase8_api_test_client_" + std::to_string(sequence.fetch_add(1U)));

    get_status_client_ = client_node_->create_client<srv::GetSystemStatus>(
      "/system/get_status");
    recover_client_ = client_node_->create_client<std_srvs::srv::Trigger>(
      "/system/recover");
    pointcloud_publisher_ = client_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/phase8_test/points", rclcpp::SensorDataQoS());
    imu_publisher_ = client_node_->create_publisher<sensor_msgs::msg::Imu>(
      "/phase8_test/imu", rclcpp::SensorDataQoS());
    status_subscription_ = client_node_->create_subscription<msg::SystemStatus>(
      "/phase8_test/system_status", 10,
      [this](const msg::SystemStatus::SharedPtr message) {
        {
          std::lock_guard<std::mutex> lock(status_mutex_);
          last_published_status_ = *message;
          status_received_ = true;
        }
        status_condition_.notify_all();
      });

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(manager_);
    executor_->add_node(client_node_);
    spin_thread_ = std::thread([this]() {executor_->spin();});

    ASSERT_TRUE(get_status_client_->wait_for_service(2s));
    ASSERT_TRUE(recover_client_->wait_for_service(2s));
  }

  void TearDown() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    if (executor_ && manager_) {
      executor_->remove_node(manager_);
    }
    if (executor_ && client_node_) {
      executor_->remove_node(client_node_);
    }
    status_subscription_.reset();
    get_status_client_.reset();
    recover_client_.reset();
    pointcloud_publisher_.reset();
    imu_publisher_.reset();
    manager_.reset();
    client_node_.reset();
    executor_.reset();
  }

  srv::GetSystemStatus::Response::SharedPtr getStatus()
  {
    auto future = get_status_client_->async_send_request(
      std::make_shared<srv::GetSystemStatus::Request>());
    if (future.wait_for(2s) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  std_srvs::srv::Trigger::Response::SharedPtr recover()
  {
    auto future = recover_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(2s) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  bool waitForState(const std::string & state, const std::chrono::milliseconds timeout = 2s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      const auto response = getStatus();
      if (response && response->success && response->status.state == state) {
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  void publishHealthySensors()
  {
    for (int index = 0; index < 4; ++index) {
      pointcloud_publisher_->publish(sensor_msgs::msg::PointCloud2());
      imu_publisher_->publish(sensor_msgs::msg::Imu());
      std::this_thread::sleep_for(30ms);
    }
  }

  std::shared_ptr<SystemManager> manager_;
  std::shared_ptr<rclcpp::Node> client_node_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  rclcpp::Client<srv::GetSystemStatus>::SharedPtr get_status_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr recover_client_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Subscription<msg::SystemStatus>::SharedPtr status_subscription_;
  std::mutex status_mutex_;
  std::condition_variable status_condition_;
  msg::SystemStatus last_published_status_;
  bool status_received_{false};
};

TEST_F(SystemManagerApiTest, GetStatusMatchesPublishedStatus)
{
  startManager(API_PROCESS_OK_CONFIG);
  {
    std::unique_lock<std::mutex> lock(status_mutex_);
    ASSERT_TRUE(status_condition_.wait_for(lock, 2s, [this]() {return status_received_;}));
  }

  const auto response = getStatus();
  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->success);

  std::lock_guard<std::mutex> lock(status_mutex_);
  EXPECT_EQ(response->status.state, last_published_status_.state);
  EXPECT_EQ(response->status.current_map, last_published_status_.current_map);
  EXPECT_EQ(response->status.error_code, last_published_status_.error_code);
  EXPECT_EQ(response->status.error_message, last_published_status_.error_message);
  EXPECT_EQ(response->status.ouster_running, last_published_status_.ouster_running);
  EXPECT_EQ(response->status.pointcloud_alive, last_published_status_.pointcloud_alive);
  EXPECT_EQ(response->status.imu_alive, last_published_status_.imu_alive);
  EXPECT_EQ(response->status.mapping_running, last_published_status_.mapping_running);
  EXPECT_EQ(response->status.localization_running, last_published_status_.localization_running);
}

TEST_F(SystemManagerApiTest, RecoverIsRejectedOutsideError)
{
  startManager(API_PROCESS_OK_CONFIG);
  const auto response = recover();
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("ERROR"), std::string::npos);
  EXPECT_TRUE(waitForState("SENSOR_STARTING"));
}

TEST_F(SystemManagerApiTest, RecoverStopsResidualMappingAndWaitsForSensors)
{
  startManager(API_PROCESS_ERROR_CONFIG);
  auto before = getStatus();
  ASSERT_NE(before, nullptr);
  ASSERT_EQ(before->status.state, "ERROR");
  ASSERT_TRUE(before->status.mapping_running);

  const auto response = recover();
  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->success);
  ASSERT_TRUE(waitForState("SENSOR_STARTING"));

  const auto after = getStatus();
  ASSERT_NE(after, nullptr);
  EXPECT_FALSE(after->status.mapping_running);
  EXPECT_TRUE(after->status.current_map.empty());
  EXPECT_TRUE(after->status.error_code.empty());
  EXPECT_TRUE(after->status.error_message.empty());
}

TEST_F(SystemManagerApiTest, RecoverReturnsToWaitModeWhenSensorsAreHealthy)
{
  startManager(API_PROCESS_ERROR_CONFIG);
  publishHealthySensors();

  const auto sensor_deadline = std::chrono::steady_clock::now() + 2s;
  bool healthy = false;
  while (std::chrono::steady_clock::now() < sensor_deadline) {
    const auto status = getStatus();
    if (status && status->status.pointcloud_alive && status->status.imu_alive) {
      healthy = true;
      break;
    }
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_TRUE(healthy);

  const auto response = recover();
  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->success);
  EXPECT_TRUE(waitForState("WAIT_MODE"));
}

TEST_F(SystemManagerApiTest, ConcurrentRecoverRequestsAcceptOnlyOne)
{
  startManager(API_PROCESS_ERROR_CONFIG);
  auto first_future = recover_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  auto second_future = recover_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(first_future.wait_for(2s), std::future_status::ready);
  ASSERT_EQ(second_future.wait_for(2s), std::future_status::ready);
  const auto first = first_future.get();
  const auto second = second_future.get();
  EXPECT_NE(first->success, second->success);
  EXPECT_TRUE(waitForState("SENSOR_STARTING"));
}

}  // namespace
}  // namespace slam_system_manager
