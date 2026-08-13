cmake_minimum_required(VERSION 3.8)
project(slam_system_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SystemStatus.msg"
  "srv/CreateMap.srv"
  "srv/DeleteMap.srv"
  "srv/GetMapList.srv"
  "srv/LoadMap.srv"
  "srv/SaveMap.srv"
  "srv/StartMapping.srv"
  "srv/StopMapping.srv"
  "srv/StartLocalization.srv"
  "srv/StopLocalization.srv"
  "srv/SetInitialPose.srv"
  DEPENDENCIES geometry_msgs
)

add_library(config_manager src/config_manager.cpp)
target_include_directories(config_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(config_manager yaml-cpp)

add_library(process_manager src/process_manager.cpp)
target_include_directories(process_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(process_manager rclcpp)
target_link_libraries(process_manager yaml-cpp)

add_library(health_monitor src/health_monitor.cpp)
target_include_directories(health_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(health_monitor rclcpp sensor_msgs)
target_link_libraries(health_monitor yaml-cpp)

add_library(map_manager src/map_manager.cpp)
target_include_directories(map_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(map_manager yaml-cpp)

add_library(mapping_adapter src/mapping_adapter.cpp)
target_include_directories(mapping_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mapping_adapter ament_index_cpp rclcpp std_srvs)
target_link_libraries(mapping_adapter process_manager yaml-cpp)

add_library(localization_adapter src/localization_adapter.cpp)
target_include_directories(localization_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(localization_adapter rclcpp)
target_link_libraries(localization_adapter process_manager yaml-cpp)

add_library(relocalization_adapter src/relocalization_adapter.cpp)
target_include_directories(relocalization_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  relocalization_adapter geometry_msgs nav_msgs rclcpp tf2 tf2_geometry_msgs)
target_link_libraries(relocalization_adapter yaml-cpp)

add_executable(system_manager_node src/system_manager.cpp)
target_include_directories(system_manager_node PRIVATE include)
ament_target_dependencies(
  system_manager_node ament_index_cpp geometry_msgs nav_msgs rclcpp sensor_msgs)
target_link_libraries(system_manager_node
  config_manager process_manager health_monitor map_manager mapping_adapter localization_adapter
  relocalization_adapter)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(system_manager_node "${cpp_typesupport_target}")

install(TARGETS
  config_manager process_manager health_monitor map_manager mapping_adapter localization_adapter
  relocalization_adapter
  system_manager_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_system_state test/test_system_state.cpp)
  target_include_directories(test_system_state PRIVATE include)

  ament_add_gtest(test_process_manager test/test_process_manager.cpp)
  target_include_directories(test_process_manager PRIVATE include)
  target_compile_definitions(test_process_manager PRIVATE
    PROCESS_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/test/process_test.yaml")
  ament_target_dependencies(test_process_manager rclcpp)
  target_link_libraries(test_process_manager process_manager)

  ament_add_gtest(test_map_manager test/test_map_manager.cpp)
  target_include_directories(test_map_manager PRIVATE include)
  target_link_libraries(test_map_manager map_manager)

  ament_add_gtest(test_mapping_adapter test/test_mapping_adapter.cpp)
  target_include_directories(test_mapping_adapter PRIVATE include)
  ament_target_dependencies(test_mapping_adapter rclcpp)
  target_link_libraries(test_mapping_adapter mapping_adapter process_manager yaml-cpp)

  ament_add_gtest(test_localization_adapter test/test_localization_adapter.cpp)
  target_include_directories(test_localization_adapter PRIVATE include)
  target_compile_definitions(test_localization_adapter PRIVATE
    LOCALIZATION_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/test/localization_test.yaml")
  ament_target_dependencies(test_localization_adapter rclcpp)
  target_link_libraries(test_localization_adapter localization_adapter process_manager yaml-cpp)

  ament_add_gtest(test_relocalization_adapter test/test_relocalization_adapter.cpp)
  target_include_directories(test_relocalization_adapter PRIVATE include)
  ament_target_dependencies(
    test_relocalization_adapter geometry_msgs nav_msgs rclcpp tf2 tf2_geometry_msgs)
  target_link_libraries(test_relocalization_adapter relocalization_adapter yaml-cpp)
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  geometry_msgs nav_msgs rclcpp rosidl_default_runtime sensor_msgs std_srvs tf2
  tf2_geometry_msgs yaml_cpp_vendor)
ament_package()
