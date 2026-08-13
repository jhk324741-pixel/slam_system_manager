#include "slam_system_manager/map_manager.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <system_error>

#include <unistd.h>

#include "yaml-cpp/yaml.h"

namespace slam_system_manager
{
namespace fs = std::filesystem;

MapManager::MapManager(std::string map_root, std::string map_frame)
: map_root_(std::move(map_root)), map_frame_(std::move(map_frame))
{
  if (!map_root_.is_absolute()) {
    throw std::invalid_argument("Map root must be an absolute path");
  }
  if (map_frame_.empty()) {
    throw std::invalid_argument("Map frame must not be empty");
  }

  std::error_code error;
  fs::create_directories(map_root_, error);
  if (error) {
    throw std::runtime_error(
            "Unable to create map root '" + map_root_.string() + "': " + error.message());
  }
  map_root_ = fs::weakly_canonical(map_root_);
  if (!fs::is_directory(map_root_)) {
    throw std::runtime_error("Map root is not a directory: " + map_root_.string());
  }
}

bool MapManager::isValidMapName(const std::string & name) noexcept
{
  static const std::regex pattern("^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$");
  return std::regex_match(name, pattern);
}

std::vector<MapMetadata> MapManager::listMaps() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<MapMetadata> maps;
  std::error_code error;
  for (fs::directory_iterator iterator(map_root_, error), end; iterator != end; iterator.increment(error)) {
    if (error) {
      throw std::runtime_error("Unable to scan map root: " + error.message());
    }
    if (!iterator->is_directory() || !isValidMapName(iterator->path().filename().string())) {
      continue;
    }
    const auto metadata_path = iterator->path() / "metadata.yaml";
    if (!fs::is_regular_file(metadata_path)) {
      continue;
    }
    maps.push_back(readMetadata(iterator->path()));
  }
  if (error) {
    throw std::runtime_error("Unable to scan map root: " + error.message());
  }
  std::sort(
    maps.begin(), maps.end(),
    [](const MapMetadata & left, const MapMetadata & right) {return left.name < right.name;});
  return maps;
}

MapMetadata MapManager::createMap(const std::string & name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto directory = mapDirectory(name);
  if (fs::exists(directory)) {
    throw std::runtime_error("Map already exists: " + name);
  }

  std::error_code error;
  if (!fs::create_directory(directory, error) || error) {
    throw std::runtime_error("Unable to create map directory: " + error.message());
  }

  MapMetadata metadata;
  metadata.name = name;
  metadata.created_at = utcNow();
  metadata.updated_at = metadata.created_at;
  metadata.frame_id = map_frame_;
  try {
    writeMetadata(directory, metadata);
  } catch (...) {
    fs::remove_all(directory, error);
    throw;
  }
  return metadata;
}

MapMetadata MapManager::markMapSaved(const std::string & name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto directory = mapDirectory(name);
  auto metadata = readMetadata(directory);
  const auto map_path = directory / metadata.map_file;
  std::error_code error;
  if (!fs::is_regular_file(map_path) || fs::file_size(map_path, error) == 0U || error) {
    throw std::runtime_error("Saved map PCD is missing or empty: " + map_path.string());
  }
  metadata.updated_at = utcNow();
  writeMetadata(directory, metadata);
  return metadata;
}

void MapManager::deleteMap(const std::string & name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto directory = mapDirectory(name);
  if (!fs::is_directory(directory)) {
    throw std::runtime_error("Map not found: " + name);
  }

  std::error_code error;
  const auto removed = fs::remove_all(directory, error);
  if (error || removed == 0U) {
    throw std::runtime_error("Unable to delete map '" + name + "': " + error.message());
  }
}

fs::path MapManager::loadMap(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto directory = mapDirectory(name);
  if (!fs::is_regular_file(directory / "metadata.yaml")) {
    throw std::runtime_error("Map metadata not found: " + name);
  }
  const auto metadata = readMetadata(directory);
  const auto map_file = directory / metadata.map_file;
  if (!fs::is_regular_file(map_file)) {
    throw std::runtime_error("Map PCD file not found: " + map_file.string());
  }
  return fs::weakly_canonical(map_file);
}

bool MapManager::mapExists(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!isValidMapName(name)) {
    return false;
  }
  const auto directory = map_root_ / name;
  return fs::is_directory(directory) && fs::is_regular_file(directory / "metadata.yaml");
}

fs::path MapManager::getMapPath(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mapDirectory(name) / "map.pcd";
}

const fs::path & MapManager::mapRoot() const noexcept
{
  return map_root_;
}

fs::path MapManager::mapDirectory(const std::string & name) const
{
  if (!isValidMapName(name)) {
    throw std::invalid_argument(
            "Invalid map name. Use 1-64 ASCII letters, digits, '_' or '-', starting with a letter or digit");
  }
  const auto directory = (map_root_ / name).lexically_normal();
  if (directory.parent_path() != map_root_) {
    throw std::invalid_argument("Map path escapes the configured map root");
  }
  return directory;
}

MapMetadata MapManager::readMetadata(const fs::path & directory) const
{
  YAML::Node root;
  try {
    root = YAML::LoadFile((directory / "metadata.yaml").string());
  } catch (const YAML::Exception & exception) {
    throw std::runtime_error(
            "Unable to read map metadata for '" + directory.filename().string() + "': " +
            exception.what());
  }

  MapMetadata metadata;
  try {
    metadata.name = root["name"].as<std::string>();
    metadata.created_at = root["created_at"].as<std::string>();
    metadata.updated_at = root["updated_at"].as<std::string>();
    metadata.map_file = root["map_file"].as<std::string>();
    metadata.frame_id = root["frame_id"].as<std::string>();
  } catch (const YAML::Exception & exception) {
    throw std::runtime_error(
            "Invalid map metadata for '" + directory.filename().string() + "': " + exception.what());
  }

  const auto directory_name = directory.filename().string();
  if (metadata.name != directory_name || !isValidMapName(metadata.name)) {
    throw std::runtime_error("Map metadata name does not match its directory: " + directory_name);
  }
  if (metadata.map_file != "map.pcd") {
    throw std::runtime_error("Map metadata contains unsupported map_file: " + metadata.map_file);
  }
  if (metadata.frame_id.empty()) {
    throw std::runtime_error("Map metadata frame_id must not be empty: " + metadata.name);
  }
  return metadata;
}

void MapManager::writeMetadata(
  const fs::path & directory, const MapMetadata & metadata) const
{
  YAML::Emitter output;
  output << YAML::BeginMap;
  output << YAML::Key << "name" << YAML::Value << metadata.name;
  output << YAML::Key << "created_at" << YAML::Value << metadata.created_at;
  output << YAML::Key << "updated_at" << YAML::Value << metadata.updated_at;
  output << YAML::Key << "map_file" << YAML::Value << metadata.map_file;
  output << YAML::Key << "frame_id" << YAML::Value << metadata.frame_id;
  output << YAML::EndMap;
  if (!output.good()) {
    throw std::runtime_error("Unable to serialize metadata for map: " + metadata.name);
  }

  const auto metadata_path = directory / "metadata.yaml";
  const auto temporary_path = directory / (".metadata.yaml.tmp." + std::to_string(getpid()));
  {
    std::ofstream stream(temporary_path, std::ios::out | std::ios::trunc);
    if (!stream) {
      throw std::runtime_error("Unable to open temporary metadata file: " + temporary_path.string());
    }
    stream << output.c_str() << '\n';
    stream.flush();
    if (!stream) {
      throw std::runtime_error("Unable to write map metadata: " + temporary_path.string());
    }
  }

  std::error_code error;
  fs::rename(temporary_path, metadata_path, error);
  if (error) {
    fs::remove(temporary_path);
    throw std::runtime_error("Unable to install map metadata: " + error.message());
  }
}

std::string MapManager::utcNow()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
  gmtime_r(&time, &utc);
  std::ostringstream output;
  output << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return output.str();
}

}  // namespace slam_system_manager
