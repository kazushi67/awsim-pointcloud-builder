cmake_minimum_required(VERSION 3.5)
project(awsim_pointcloud_builder)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL 1.1 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(geometory_msgs REQUIRED)

# find_package(std_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (awsim_pointcloud_builder src/awsim_pointcloud_builder.cpp)
target_link_libraries (awsim_pointcloud_builder ${PCL_LIBRARIES})
ament_target_dependencies(awsim_pointcloud_builder rclcpp pcl_conversions pcl_ros sensor_msgs)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(TARGETS
  awsim_pointcloud_builder
  # sensor_msgs
  # pcl_ros
  # geometory_msgs
  DESTINATION lib/${PROJECT_NAME})
ament_package()
