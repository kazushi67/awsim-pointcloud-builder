#include <cstdio>
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

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

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
      rclcpp::QoS qos(rclcpp::KeepLast(5));
      qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
      qos.durability(rclcpp::DurabilityPolicy::Volatile);
      save_path_ = this->declare_parameter<string>("save_path");
      x_offset_ = this->declare_parameter<double>("offset_x", 0.9);
      y_offset_ = this->declare_parameter<double>("offset_y", 0.0);
      z_offset_ = this->declare_parameter<double>("offset_z", 2.0);
      roll_offset_ = this->declare_parameter<double>("offset_roll", -0.001);
      pitch_offset_ = this->declare_parameter<double>("offset_pitch", 0.015);
      yaw_offset_ = this->declare_parameter<double>("offset_yaw", -0.0364);
      sub_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "point_cloud", qos, std::bind(&AwsimMapBuilder::pointcloud_callback, this, std::placeholders::_1));

      sub_ground_truth_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "ground_truth", 10, std::bind(&AwsimMapBuilder::gt_callback, this, std::placeholders::_1));

      timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&AwsimMapBuilder::timer_callback, this));
    }

    void public_transformPointCloud(){
      transformPointCloud();
    }

  private:
    std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> pointcloud_array_;
    std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> pointcloud_in_;
    vector<geometry_msgs::msg::PoseStamped> gt_array_;
    vector<geometry_msgs::msg::PoseStamped> gt_in_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_out_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pointcloud_out_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    string save_path_;
    double x_offset_;
    double y_offset_; 
    double z_offset_;
    double roll_offset_;
    double pitch_offset_;
    double yaw_offset_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_ground_truth_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool isSuscribe = false;
    bool isReady = true;

    void pointcloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      isSuscribe = true;
      isReady = false;
      pointcloud_array_.push_back(msg);
    } 

    void gt_callback(const geometry_msgs::msg::PoseStamped & msg)
    {
      gt_array_.push_back(msg);
    }

    void transformPointCloud()
    {
      // check Error(point cloud is empty)
      if(pointcloud_array_.size() == 0){
        RCLCPP_ERROR(this->get_logger(), "Subscribe PCD-topic num is 0");  
        return; 
      }

      // interpolate
      for(size_t i = 0; i < pointcloud_array_.size(); i++){
        for(size_t h = i; h < gt_array_.size(); h++){
          if(pointcloud_array_.at(i)->header.stamp == gt_array_.at(h).header.stamp){
            gt_in_.push_back(gt_array_.at(h));
            pointcloud_in_.push_back(pointcloud_array_.at(i));
          }
        }
      }

      // calculate transform
      tf2::Quaternion q;
      for(size_t j = 0; j < pointcloud_in_.size(); j++){
        double roll,pitch,yaw;
        q.setValue(
          gt_in_.at(j).pose.orientation.x,
          gt_in_.at(j).pose.orientation.y,
          gt_in_.at(j).pose.orientation.z,
          gt_in_.at(j).pose.orientation.w
        );
        tf2::Matrix3x3 matrix(q);
        matrix.getRPY(roll,pitch,yaw);
        Eigen::Affine3f transBaseLink = pcl::getTransformation(
          gt_in_.at(j).pose.position.x, 
          gt_in_.at(j).pose.position.y, 
          gt_in_.at(j).pose.position.z, 
          roll, pitch, yaw
        );

        //**change to sensor-frame (awsim sensor-kit)**//
        Eigen::Affine3f transBaseLink2SensorFrame = pcl::getTransformation(
          x_offset_, 
          y_offset_, 
          z_offset_, 
          roll_offset_, 
          pitch_offset_, 
          yaw_offset_
        );

        Eigen::Affine3f transSensorFrame = transBaseLink * transBaseLink2SensorFrame;

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud_(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pointcloud_in_.at(j), *pcl_pointcloud_);

        // #pragma omp parallel for num_threads(numberOfCores)
        for (size_t i = 0; i < pcl_pointcloud_->size(); ++i)
        {
          pcl::PointXYZ input_point_ = pcl_pointcloud_->points[i];
          pcl::PointXYZ point;
          point.x = transSensorFrame(0,0) * input_point_.x + transSensorFrame(0,1) * input_point_.y + transSensorFrame(0,2) * input_point_.z + transSensorFrame(0,3);
          point.y = transSensorFrame(1,0) * input_point_.x + transSensorFrame(1,1) * input_point_.y + transSensorFrame(1,2) * input_point_.z + transSensorFrame(1,3);
          point.z = transSensorFrame(2,0) * input_point_.x + transSensorFrame(2,1) * input_point_.y + transSensorFrame(2,2) * input_point_.z + transSensorFrame(2,3);
          pointcloud_out_->push_back(point);
        }
      }

      // downsample pointcloud
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud (pointcloud_out_);
      vg.setLeafSize (1.0, 1.0, 1.0);
      vg.filter (*filtered_pointcloud_out_);

      // save pcd-file
      try{
        pcl::io::savePCDFileBinary (save_path_, *filtered_pointcloud_out_);
        RCLCPP_INFO(this->get_logger(), "Success to save Map.");
      } catch (pcl::IOException& e){
        RCLCPP_ERROR(this->get_logger(), "Failed to save Map.");  
      }

      return;
    }

    void timer_callback()
    {
      if(isReady == true){
        RCLCPP_INFO(this->get_logger(), "Ready to subscribe PCD-topic.");  
        return;
      }

      if(isReady == false && isSuscribe == false){
        RCLCPP_INFO(this->get_logger(), "Stop and make pointcloud_map.");
        rclcpp::shutdown();
        return;
      }
      
      RCLCPP_INFO(this->get_logger(), "Subscribe PCD-topic.");
      isSuscribe = false;
    }
};
