#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "slam_system_manager/localization_adapter.hpp"
#include "slam_system_manager/process_manager.hpp"

namespace slam_system_manager
{
namespace fs = std::filesystem;

class LocalizationAdapterTest : public ::testing::Test
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
      ("slam_system_manager_localization_adapter_test_" + std::to_string(getpid()));
    fs::remove_all(root_);
    fs::create_directories(root_ / "test_map");
    map_path_ = root_ / "test_map" / "map.pcd";
    std::ofstream map(map_path_);
    map << "VERSION .7\nDATA ascii\n0 0 0\n";
  }

  void TearDown() override
  {
    fs::remove_all(root_);
  }

  fs::path root_;
  fs::path map_path_;
};

TEST_F(LocalizationAdapterTest, StartsAndStopsWithValidMap)
{
  auto node = std::make_shared<rclcpp::Node>("localization_adapter_test");
  ProcessManager process_manager(node->get_logger());
  process_manager.loadConfig(LOCALIZATION_TEST_CONFIG);
  LocalizationAdapter adapter(*node, process_manager, LOCALIZATION_TEST_CONFIG);

  std::string error;
  ASSERT_TRUE(adapter.start("test_map", map_path_, &error)) << error;
  EXPECT_TRUE(adapter.isRunning());
  EXPECT_FALSE(adapter.start("test_map", map_path_, &error));
  EXPECT_NE(error.find("already running"), std::string::npos);
  ASSERT_TRUE(adapter.stop(&error)) << error;
  EXPECT_FALSE(adapter.isRunning());
}

TEST_F(LocalizationAdapterTest, RejectsMissingMap)
{
  auto node = std::make_shared<rclcpp::Node>("localization_adapter_missing_map_test");
  ProcessManager process_manager(node->get_logger());
  process_manager.loadConfig(LOCALIZATION_TEST_CONFIG);
  LocalizationAdapter adapter(*node, process_manager, LOCALIZATION_TEST_CONFIG);

  std::string error;
  EXPECT_FALSE(adapter.start("missing", root_ / "missing" / "map.pcd", &error));
  EXPECT_NE(error.find("does not exist"), std::string::npos);
}

TEST_F(LocalizationAdapterTest, RejectsEmptyAndUnsafeMapPaths)
{
  auto node = std::make_shared<rclcpp::Node>("localization_adapter_invalid_map_test");
  ProcessManager process_manager(node->get_logger());
  process_manager.loadConfig(LOCALIZATION_TEST_CONFIG);
  LocalizationAdapter adapter(*node, process_manager, LOCALIZATION_TEST_CONFIG);

  std::string error;
  std::ofstream(map_path_, std::ios::trunc).close();
  EXPECT_FALSE(adapter.start("empty", map_path_, &error));
  EXPECT_NE(error.find("empty or unreadable"), std::string::npos);
  EXPECT_FALSE(adapter.start("relative", fs::path("map.pcd"), &error));
  EXPECT_NE(error.find("absolute path"), std::string::npos);
}

}  // namespace slam_system_manager
