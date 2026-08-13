#include <gtest/gtest.h>

#include "slam_system_manager/system_state.hpp"

namespace slam_system_manager
{

TEST(SystemState, ConvertsEveryStateToAStableName)
{
  EXPECT_EQ(toString(SystemState::BOOT), "BOOT");
  EXPECT_EQ(toString(SystemState::WAIT_MODE), "WAIT_MODE");
  EXPECT_EQ(toString(SystemState::LOCALIZED), "LOCALIZED");
  EXPECT_EQ(toString(SystemState::SHUTDOWN), "SHUTDOWN");
}

TEST(SystemState, AllowsNominalMappingFlow)
{
  EXPECT_TRUE(canTransition(SystemState::WAIT_MODE, SystemState::MAPPING_STARTING));
  EXPECT_TRUE(canTransition(SystemState::MAPPING_STARTING, SystemState::MAPPING));
  EXPECT_TRUE(canTransition(SystemState::MAPPING, SystemState::MAP_SAVING));
  EXPECT_TRUE(canTransition(SystemState::MAP_SAVING, SystemState::WAIT_MODE));
}

TEST(SystemState, AllowsSensorReadinessToBeLostWhileIdle)
{
  EXPECT_TRUE(canTransition(SystemState::SENSOR_STARTING, SystemState::SENSOR_READY));
  EXPECT_TRUE(canTransition(SystemState::SENSOR_READY, SystemState::SENSOR_STARTING));
}

TEST(SystemState, AllowsNominalLocalizationFlow)
{
  EXPECT_TRUE(canTransition(SystemState::WAIT_MODE, SystemState::LOCALIZATION_STARTING));
  EXPECT_TRUE(canTransition(SystemState::LOCALIZATION_STARTING, SystemState::RELOCALIZING));
  EXPECT_TRUE(canTransition(SystemState::RELOCALIZING, SystemState::LOCALIZED));
  EXPECT_TRUE(canTransition(SystemState::LOCALIZED, SystemState::WAIT_MODE));
}

TEST(SystemState, RejectsConflictingAndInvalidTransitions)
{
  EXPECT_FALSE(canTransition(SystemState::MAPPING, SystemState::LOCALIZATION_STARTING));
  EXPECT_FALSE(canTransition(SystemState::LOCALIZED, SystemState::MAPPING_STARTING));
  EXPECT_FALSE(canTransition(SystemState::WAIT_MODE, SystemState::LOCALIZED));
  EXPECT_FALSE(canTransition(SystemState::SHUTDOWN, SystemState::BOOT));
  EXPECT_FALSE(canTransition(SystemState::MAPPING, SystemState::MAPPING));
}

TEST(SystemState, AllowsConservativeRecoveryFlow)
{
  EXPECT_TRUE(canTransition(SystemState::ERROR, SystemState::RECOVERING));
  EXPECT_TRUE(canTransition(SystemState::RECOVERING, SystemState::WAIT_MODE));
  EXPECT_TRUE(canTransition(SystemState::RECOVERING, SystemState::SENSOR_STARTING));
  EXPECT_TRUE(canTransition(SystemState::RECOVERING, SystemState::ERROR));
  EXPECT_FALSE(canTransition(SystemState::WAIT_MODE, SystemState::RECOVERING));
  EXPECT_FALSE(canTransition(SystemState::RECOVERING, SystemState::MAPPING));
}

}  // namespace slam_system_manager
