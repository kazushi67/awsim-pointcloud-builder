cmake_minimum_required(VERSION 3.5)
project(awsim_pointcloud_builder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
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

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable (awsim_pointcloud_builder src/awsim_pointcloud_builder.cpp)
ament_target_dependencies(awsim_pointcloud_builder rclcpp pcl_conversions pcl_ros sensor_msgs)

include_directories(include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  awsim_pointcloud_builder
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()