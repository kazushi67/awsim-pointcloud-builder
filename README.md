# awsim-pointcloud-builder

## Discription
This pkg make pointcloud-map with [AWSIM](https://tier4.github.io/AWSIM/). When create custom enviroment within AWSIM, this pkg helps make pointcloud for [Autoware](https://github.com/autowarefoundation/autoware) 

This pkg is based on ros2-humble, pcl

https://docs.ros.org/en/humble/index.html

https://pointclouds.org/

## Input/Output
    INPUT ： Pointcloud-Topic, PoseGT-Topic
    OUTPUT： .pcd-format-map file


## How to use
1. Prepare AWSIM-binary [follwing here](https://tier4.github.io/AWSIM/)
2. Build this pkg from this source
3. Launch this pkg after running AWSIM  
    ```bash
    ros2 launch awsim_pointcloud_builder awsim_pointcloud_builder.launch.xml save_path:=<save_map_path>
    ```

4. Stop AWSIM. This pkg stop automatically After stopping AWSIM.