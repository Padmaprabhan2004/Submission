cmake_minimum_required(VERSION 3.8)
project(obj_seg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(traffic_segmentation src/traffic_segmentation.cpp)
ament_target_dependencies(traffic_segmentation rclcpp sensor_msgs pcl_conversions visualization_msgs)
target_link_libraries(traffic_segmentation ${PCL_LIBRARIES})

install(TARGETS
  pcd_import traffic_segmentation
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

