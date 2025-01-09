#include "awsim_pointcloud_builder/awsim_pointcloud_builder.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto instance = std::make_shared<AwsimMapBuilder>();
  rclcpp::spin(instance);
  instance->public_transformPointCloud();
  return 0;
}
