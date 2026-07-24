cmake_minimum_required(VERSION 3.5)
project(akm_vehicle_sensor_kit_launch)

find_package(ament_cmake_auto REQUIRED)
find_package(CUDA REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(pointcloud_xyzirc_conversion SHARED
  src/pointcloud_xyzirc_conversion.cpp
)

ament_auto_add_library(pointcloud_xyzirc_adapter_component SHARED
  src/pointcloud_xyzirc_adapter.cpp
)
target_link_libraries(pointcloud_xyzirc_adapter_component pointcloud_xyzirc_conversion)
target_include_directories(pointcloud_xyzirc_adapter_component SYSTEM PRIVATE ${CUDA_INCLUDE_DIRS})
target_link_libraries(pointcloud_xyzirc_adapter_component ${CUDA_LIBRARIES})

rclcpp_components_register_node(pointcloud_xyzirc_adapter_component
  PLUGIN "akm_vehicle_sensor_kit_launch::PointCloudXyzircAdapter"
  EXECUTABLE pointcloud_xyzirc_adapter
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_pointcloud_xyzirc_conversion
    test/test_pointcloud_xyzirc_conversion.cpp
  )
  target_include_directories(test_pointcloud_xyzirc_conversion PRIVATE include)
  ament_target_dependencies(test_pointcloud_xyzirc_conversion sensor_msgs)
  target_link_libraries(test_pointcloud_xyzirc_conversion pointcloud_xyzirc_conversion)
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  data
  config
  rviz
)
