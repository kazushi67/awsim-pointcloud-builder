#include <cstdio>
//std_lib
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <complex>
#include <stdio.h>
#include <iomanip>
#include <chrono>
#include <functional>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>

//ros_lib
// #include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
//#include <sensor_msgs/msg/string>
#include <tf2/LinearMath/Matrix3x3.h>
//ros_pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

using namespace std;
class AwsimMapBuilder : public rclcpp::Node
{
  public:
    AwsimMapBuilder(): Node("pointcloud_publisher")
    {
      rclcpp::QoS custom_qos(rclcpp::KeepLast(5));
      custom_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
      custom_qos.durability(rclcpp::DurabilityPolicy::Volatile);
      save_path_ = this->declare_parameter<string>("save_path");
      sub_point_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "point_cloud", custom_qos, std::bind(&AwsimMapBuilder::pointcloud_callback, this, std::placeholders::_1));

      sub_ground_truth_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "ground_truth", 10, std::bind(&AwsimMapBuilder::gt_callback, this, std::placeholders::_1));

      timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&AwsimMapBuilder::timer_callback, this));
    }

    void public_transformPointCloud(){
      transformPointCloud();
    }

  private:
    std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> point_cloud_array;
    std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> point_cloud_in;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_out = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_point_cloud_out = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    vector<geometry_msgs::msg::PoseStamped> ground_truth_in;
    string save_path_;

    void pointcloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      is_finish = false;
      point_cloud_array.push_back(msg);
    } 
   
    vector<geometry_msgs::msg::PoseStamped> gt_array;

    void gt_callback(const geometry_msgs::msg::PoseStamped & msg)
    {
      gt_array.push_back(msg);
    }

    void transformPointCloud()
    {
      // check Error(point cloud is empty)
      if(point_cloud_array.size() == 0){
        RCLCPP_ERROR(this->get_logger(), "Subscrive PCD-topic is 0");  
        return; 
      }

      // interpolate
      for(size_t i = 0; i < point_cloud_array.size(); i++){
        for(size_t h = i; h < gt_array.size(); h++){
          if(point_cloud_array.at(i)->header.stamp == gt_array.at(h).header.stamp){
            ground_truth_in.push_back(gt_array.at(h));
            point_cloud_in.push_back(point_cloud_array.at(i));
          }
        }
      }

      // calculate transform
      tf2::Quaternion q;
      for(size_t j = 0; j < point_cloud_in.size(); j++){
        double roll,pitch,yaw;
        q.setValue(
          ground_truth_in.at(j).pose.orientation.x,
          ground_truth_in.at(j).pose.orientation.y,
          ground_truth_in.at(j).pose.orientation.z,
          ground_truth_in.at(j).pose.orientation.w
        );
        tf2::Matrix3x3 matrix(q);
        matrix.getRPY(roll,pitch,yaw);
        Eigen::Affine3f transBaseLink = pcl::getTransformation(
          ground_truth_in.at(j).pose.position.x, 
          ground_truth_in.at(j).pose.position.y, 
          ground_truth_in.at(j).pose.position.z, 
          roll, pitch, yaw
        );

        //**change to sensor-frame (awsim sensor-kit)**//
        double x_offset = 0.9;
        double y_offset =  0.0;
        double z_offset =  2.0;
        double roll_offset =  -0.001;
        double pitch_offset =  0.015;
        double yaw_offset = -0.0364;

        Eigen::Affine3f transSensorFrame = pcl::getTransformation(
          x_offset, 
          y_offset, 
          z_offset , 
          roll_offset, 
          pitch_offset, 
          yaw_offset
        );

        Eigen::Affine3f trans = transBaseLink * transSensorFrame;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn_pcl(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*point_cloud_in.at(j), *cloudIn_pcl);

        // #pragma omp parallel for num_threads(numberOfCores)
        for (size_t i = 0; i < cloudIn_pcl->size(); ++i)
        {
          pcl::PointXYZ input_point = cloudIn_pcl->points[i];
          pcl::PointXYZ point;
          point.x = trans(0,0) * input_point.x + trans(0,1) * input_point.y + trans(0,2) * input_point.z + trans(0,3);
          point.y = trans(1,0) * input_point.x + trans(1,1) * input_point.y + trans(1,2) * input_point.z + trans(1,3);
          point.z = trans(2,0) * input_point.x + trans(2,1) * input_point.y + trans(2,2) * input_point.z + trans(2,3);
          point_cloud_out->push_back(point);
        }
      }

      // downsample pointcloud
      pcl::VoxelGrid<pcl::PointXYZ> VG;
      VG.setInputCloud (point_cloud_out);
      VG.setLeafSize (1.0, 1.0, 1.0);
      VG.filter (*filtered_point_cloud_out);

      // save pcd-file
      try{
        pcl::io::savePCDFileBinary (save_path_, *filtered_point_cloud_out);
        RCLCPP_INFO(this->get_logger(), "Success to save Map");
      } catch (pcl::IOException& e){
        RCLCPP_ERROR(this->get_logger(), "Failed to save Map");  
      }

      return;
    }

    void timer_callback()
    {
      if(is_finish == true){
        RCLCPP_ERROR(this->get_logger(), "Don't subscrive PCD-topic");  
        rclcpp::shutdown();
        return;
      }
      
      RCLCPP_INFO(this->get_logger(), "subscrive PCD-topic");
      is_finish=true;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_ground_truth_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool is_finish = true;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto instance = std::make_shared<AwsimMapBuilder>();
  rclcpp::spin(instance);
  instance->public_transformPointCloud();
  return 0;
}
