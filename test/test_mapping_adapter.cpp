#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "slam_system_manager/mapping_adapter.hpp"
#include "slam_system_manager/process_manager.hpp"
#include "yaml-cpp/yaml.h"

namespace slam_system_manager
{
namespace fs = std::filesystem;

class MappingAdapterTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    root_ = fs::temp_directory_path() /
      ("slam_system_manager_mapping_adapter_test_" + std::to_string(getpid()));
    fs::remove_all(root_);
    fs::create_directories(root_ / "map_a");

    template_path_ = root_ / "ouster64.yaml";
    {
      std::ofstream output(template_path_);
      output << "/**:\n"
             << "  ros__parameters:\n"
             << "    map_file_path: ./old.pcd\n"
             << "    pcd_save:\n"
             << "      pcd_save_en: false\n"
             << "      interval: -1\n";
    }

    process_config_path_ = root_ / "process.yaml";
    {
      std::ofstream output(process_config_path_);
      output << "processes:\n"
             << "  mapping:\n"
             << "    command: /bin/sleep 30\n"
             << "    auto_start: false\n"
             << "    sigint_timeout_sec: 0.2\n"
             << "    sigterm_timeout_sec: 0.2\n"
             << "mapping_adapter:\n"
             << "  template_config: '" << template_path_.string() << "'\n"
             << "  runtime_config_name: mapping_runtime.yaml\n"
             << "  startup_timeout_sec: 1.0\n"
             << "  save_service:\n"
             << "    name: /mapping_adapter_test/map_save\n"
             << "    type: std_srvs/srv/Trigger\n"
             << "    timeout_sec: 1.0\n";
    }
  }

  void TearDown() override
  {
    fs::remove_all(root_);
  }

  fs::path root_;
  fs::path template_path_;
  fs::path process_config_path_;
};

TEST_F(MappingAdapterTest, GeneratesPerMapRuntimeConfig)
{
  auto node = std::make_shared<rclcpp::Node>("mapping_adapter_test");
  ProcessManager process_manager(node->get_logger());
  process_manager.loadConfig(process_config_path_.string());
  MappingAdapter adapter(*node, process_manager, process_config_path_.string());

  const auto map_path = root_ / "map_a" / "map.pcd";
  const auto runtime_path = adapter.prepareRuntimeConfig(map_path);
  ASSERT_EQ(runtime_path, root_ / "map_a" / "mapping_runtime.yaml");

  const auto root = YAML::LoadFile(runtime_path.string());
  const auto parameters = root["/**"]["ros__parameters"];
  EXPECT_EQ(parameters["map_file_path"].as<std::string>(), map_path.string());
  EXPECT_TRUE(parameters["pcd_save"]["pcd_save_en"].as<bool>());
  EXPECT_EQ(parameters["pcd_save"]["interval"].as<int>(), -1);
}

TEST_F(MappingAdapterTest, RejectsUnsafeOutputAndPreservesTemplate)
{
  auto node = std::make_shared<rclcpp::Node>("mapping_adapter_validation_test");
  ProcessManager process_manager(node->get_logger());
  process_manager.loadConfig(process_config_path_.string());
  MappingAdapter adapter(*node, process_manager, process_config_path_.string());

  EXPECT_THROW(adapter.prepareRuntimeConfig(root_ / "map_a" / "wrong.pcd"), std::invalid_argument);
  EXPECT_THROW(adapter.prepareRuntimeConfig("relative/map.pcd"), std::invalid_argument);

  const auto original = YAML::LoadFile(template_path_.string());
  EXPECT_EQ(
    original["/**"]["ros__parameters"]["map_file_path"].as<std::string>(),
    "./old.pcd");
  EXPECT_FALSE(original["/**"]["ros__parameters"]["pcd_save"]["pcd_save_en"].as<bool>());
}

}  // namespace slam_system_manager
