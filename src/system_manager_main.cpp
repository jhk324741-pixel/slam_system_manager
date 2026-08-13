#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "slam_system_manager/system_manager.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<slam_system_manager::SystemManager>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("system_manager"), "[SYSTEM] Startup failed: %s",
      exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
