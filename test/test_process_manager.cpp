#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "rclcpp/logger.hpp"
#include "slam_system_manager/process_manager.hpp"

namespace slam_system_manager
{
namespace
{

std::optional<int> waitForExitCode(ProcessManager & manager, const std::string & name)
{
  for (int attempt = 0; attempt < 30; ++attempt) {
    const auto code = manager.lastExitCode(name);
    if (code.has_value()) {
      return code;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return std::nullopt;
}

std::unique_ptr<ProcessManager> makeManager()
{
  auto manager = std::make_unique<ProcessManager>(rclcpp::get_logger("process_manager_test"));
  manager->loadConfig(PROCESS_TEST_CONFIG);
  return manager;
}

}  // namespace

TEST(ProcessManager, StartsStopsAndRejectsDuplicateStart)
{
  auto manager = makeManager();
  std::string error;
  ASSERT_TRUE(manager->startProcess("long_running", {}, &error)) << error;
  EXPECT_TRUE(manager->isProcessRunning("long_running"));
  EXPECT_FALSE(manager->startProcess("long_running", {}, &error));
  EXPECT_NE(error.find("already running"), std::string::npos);
  ASSERT_TRUE(manager->stopProcess("long_running", &error)) << error;
  EXPECT_FALSE(manager->isProcessRunning("long_running"));
  ASSERT_TRUE(manager->lastExitCode("long_running").has_value());
}

TEST(ProcessManager, CapturesNormalExitCode)
{
  auto manager = makeManager();
  std::string error;
  ASSERT_TRUE(manager->startProcess("short_lived", {}, &error)) << error;
  const auto code = waitForExitCode(*manager, "short_lived");
  ASSERT_TRUE(code.has_value());
  EXPECT_EQ(*code, 7);
}

TEST(ProcessManager, ReplacesCommandTemplatesAndRejectsMissingValues)
{
  auto manager = makeManager();
  std::string error;
  EXPECT_FALSE(manager->startProcess("templated", {}, &error));
  EXPECT_NE(error.find("{seconds}"), std::string::npos);
  ASSERT_TRUE(
    manager->startProcess("templated", {{"seconds", "30"}}, &error)) << error;
  EXPECT_TRUE(manager->isProcessRunning("templated"));
  EXPECT_TRUE(manager->stopProcess("templated", &error)) << error;
}

TEST(ProcessManager, UsesDefaultSubstitutionsAndSupportsAutoStartExclusions)
{
  auto manager = makeManager();
  std::string error;
  ASSERT_TRUE(
    manager->startAutoStartProcesses(&error, {"auto_templated"})) << error;
  EXPECT_FALSE(manager->isProcessRunning("auto_templated"));

  const auto auto_start_names = manager->autoStartProcessNames();
  EXPECT_NE(
    std::find(auto_start_names.begin(), auto_start_names.end(), "auto_templated"),
    auto_start_names.end());

  ASSERT_TRUE(manager->startAutoStartProcesses(&error)) << error;
  EXPECT_TRUE(manager->isProcessRunning("auto_templated"));
  ASSERT_TRUE(manager->stopProcess("auto_templated", &error)) << error;

  ASSERT_TRUE(
    manager->startProcess("auto_templated", {{"seconds", "20"}}, &error)) << error;
  EXPECT_TRUE(manager->isProcessRunning("auto_templated"));
  EXPECT_TRUE(manager->stopProcess("auto_templated", &error)) << error;
}

TEST(ProcessManager, EscalatesToSigkillWhenProcessIgnoresStopSignals)
{
  auto manager = makeManager();
  std::string error;
  ASSERT_TRUE(manager->startProcess("ignores_stop", {}, &error)) << error;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ASSERT_TRUE(manager->stopProcess("ignores_stop", &error)) << error;
  const auto code = manager->lastExitCode("ignores_stop");
  ASSERT_TRUE(code.has_value());
  EXPECT_EQ(*code, 128 + SIGKILL);
}

}  // namespace slam_system_manager
