#ifndef SLAM_SYSTEM_MANAGER__MAP_MANAGER_HPP_
#define SLAM_SYSTEM_MANAGER__MAP_MANAGER_HPP_

#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

namespace slam_system_manager
{

struct MapMetadata
{
  std::string name;
  std::string created_at;
  std::string updated_at;
  std::string map_file{"map.pcd"};
  std::string frame_id{"map"};
};

class MapManager
{
public:
  MapManager(std::string map_root, std::string map_frame);

  static bool isValidMapName(const std::string & name) noexcept;

  std::vector<MapMetadata> listMaps() const;
  MapMetadata createMap(const std::string & name);
  MapMetadata markMapSaved(const std::string & name);
  void deleteMap(const std::string & name);
  std::filesystem::path loadMap(const std::string & name) const;
  bool mapExists(const std::string & name) const;
  std::filesystem::path getMapPath(const std::string & name) const;
  const std::filesystem::path & mapRoot() const noexcept;

private:
  std::filesystem::path mapDirectory(const std::string & name) const;
  MapMetadata readMetadata(const std::filesystem::path & directory) const;
  void writeMetadata(
    const std::filesystem::path & directory, const MapMetadata & metadata) const;
  static std::string utcNow();

  mutable std::mutex mutex_;
  std::filesystem::path map_root_;
  std::string map_frame_;
};

}  // namespace slam_system_manager

#endif  // SLAM_SYSTEM_MANAGER__MAP_MANAGER_HPP_
