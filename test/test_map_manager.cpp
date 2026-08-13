#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include <unistd.h>

#include "slam_system_manager/map_manager.hpp"

namespace slam_system_manager
{
namespace fs = std::filesystem;

class MapManagerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    root_ = fs::temp_directory_path() /
      ("slam_system_manager_map_test_" + std::to_string(getpid()));
    fs::remove_all(root_);
  }

  void TearDown() override
  {
    fs::remove_all(root_);
  }

  fs::path root_;
};

TEST_F(MapManagerTest, CreatesRootMapAndMetadata)
{
  MapManager manager(root_.string(), "map");
  const auto metadata = manager.createMap("factory_A");

  EXPECT_EQ(metadata.name, "factory_A");
  EXPECT_EQ(metadata.map_file, "map.pcd");
  EXPECT_EQ(metadata.frame_id, "map");
  EXPECT_TRUE(fs::is_regular_file(root_ / "factory_A" / "metadata.yaml"));
  EXPECT_TRUE(manager.mapExists("factory_A"));
  EXPECT_EQ(manager.getMapPath("factory_A"), root_ / "factory_A" / "map.pcd");
}

TEST_F(MapManagerTest, RejectsTraversalAndUnsafeNames)
{
  MapManager manager(root_.string(), "map");
  EXPECT_FALSE(MapManager::isValidMapName("../../etc/passwd"));
  EXPECT_FALSE(MapManager::isValidMapName("map/name"));
  EXPECT_FALSE(MapManager::isValidMapName(" map"));
  EXPECT_FALSE(MapManager::isValidMapName(""));
  EXPECT_THROW(manager.createMap("../../etc/passwd"), std::invalid_argument);
  EXPECT_THROW(manager.getMapPath("map/name"), std::invalid_argument);
}

TEST_F(MapManagerTest, ListsLoadsAndDeletesMaps)
{
  MapManager manager(root_.string(), "map");
  manager.createMap("warehouse_01");
  manager.createMap("factory_A");
  {
    std::ofstream map_file(root_ / "factory_A" / "map.pcd");
    map_file << "VERSION .7\n";
  }

  const auto maps = manager.listMaps();
  ASSERT_EQ(maps.size(), 2U);
  EXPECT_EQ(maps[0].name, "factory_A");
  EXPECT_EQ(maps[1].name, "warehouse_01");
  EXPECT_EQ(manager.loadMap("factory_A"), fs::weakly_canonical(root_ / "factory_A" / "map.pcd"));
  EXPECT_THROW(manager.loadMap("warehouse_01"), std::runtime_error);

  manager.deleteMap("warehouse_01");
  EXPECT_FALSE(manager.mapExists("warehouse_01"));
  EXPECT_THROW(manager.deleteMap("warehouse_01"), std::runtime_error);
}

TEST_F(MapManagerTest, IgnoresDirectoriesWithoutMetadata)
{
  MapManager manager(root_.string(), "map");
  fs::create_directory(root_ / "unmanaged");
  EXPECT_TRUE(manager.listMaps().empty());
}

TEST_F(MapManagerTest, MarksNonEmptyPcdAsSaved)
{
  MapManager manager(root_.string(), "map");
  const auto original = manager.createMap("saved_map");
  EXPECT_THROW(manager.markMapSaved("saved_map"), std::runtime_error);

  {
    std::ofstream map_file(root_ / "saved_map" / "map.pcd");
    map_file << "VERSION .7\n";
  }
  const auto updated = manager.markMapSaved("saved_map");
  EXPECT_EQ(updated.name, "saved_map");
  EXPECT_GE(updated.updated_at, original.updated_at);
  EXPECT_EQ(manager.loadMap("saved_map"), fs::weakly_canonical(root_ / "saved_map" / "map.pcd"));
}

}  // namespace slam_system_manager
