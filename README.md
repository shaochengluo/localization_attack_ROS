# localization_attack_ros
Localization attack virtual testbed in ROS+Gazebo

## Installation

### 1. Install ROS
1. Follow the instructions on the [ROS installation instructions](http://wiki.ros.org/noetic/Installation) website to install ROS. If you are unfamiliar with ROS, its worth taking some of the [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials). We use ROS noetic developed for Ubuntu 20.04. Using other ROS versions may require some changes

Note: If you plan to integrate with a TI radar sensor (using the CPSL_TI_Radar_ROS package), you will also need to install the [CPSL_TI_Radar](https://github.com/davidmhunt/CPSL_TI_Radar.git) modules as well.

## Adding localization_attack_ros packages to catkin workspace

If you haven't done so already, creat a catkin workspace. In the following steps, replace "catkin_ws" with the name of your catkin workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```

1. To add the two packages to the catkin workspace, perform the following commands
```
cd catkin_ws/src/
git clone --recurse-submodules https://github.com/shaochengluo/localization_attack_ros.git
```
Here, catkin_ws is the path to your catkin workspace

If you forgot to perform the --recurse-submodules when cloning the repository, you can use the following command to load the necessary submodules
```
git submodule update --init --recursive
```

2. Next, install all of the required ros dependencies using the following commands.

First, install all other dependencies required by the CPSL_ROS_Sensors package
```
cd ~/catkin_ws
rosdep install --from-paths ~/catkin_ws/src/localization_attack_ros/ --ignore-src -y --rosdistro=noetic
```
Note: we build this package for ROS noetic, if you are using a different ROS distribution, some of the above packages may have changed

3. Next, build the ROS nodes in your catkin workspace using the following commands:
```
cd ~/catkin_ws
catkin_make
```

4. Finally, source the setup.bash file so that ROS can find the nodes and the messages
```
source devel/setup.bash
```

## Tutorials

### 1. SLAM/Mapping in the simulated environment (Gazebo House)

1.1 Launch Simulation World, in terminal 1:
```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_house.launch
```
Run SLAM Node, in terminal 2:
```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```
Run Teleoperation Node, in terminal 3:
```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

 Control Your TurtleBot3!
 ---------------------------
 Moving around:
        w
   a    s    d
        x

 w/x : increase/decrease linear velocity
 a/d : increase/decrease angular velocity
 space key, s : force stop

 CTRL-C to quit
```
Save Map, in terminal 4:
```
rosrun map_server map_saver -f ~/map
```


For its research efforts, the CPSL implemented a custom map server that additionally publishes a dynamic map that can incorporate updates from a ROS [sensor/msgs/PointCloud2](https://docs.ros.org/en/jade/api/sensor_msgs/html/msg/PointCloud2.html) or [jsk_recognition_msgs/BoundingBoxArray](https://docs.ros.org/en/jade/api/jsk_recognition_msgs/html/msg/BoundingBoxArray.html) messages. To ease the use of this new node, we created a [map_server_sensor_updates.launch](/CPSL_ROS_navigation_ROS_fork/map_server/launch/map_server_sensor_updates.launch) file. The defaults can be modified fairly easily in the launch file or via the terminal. As a quick way of getting started, the following command can be used to launch a map from a .yaml file (for a .pgm map). **NOTE: UPDATE THE PATH TO THE MAP WITH THE DESIRED MAP**
```
cd catkin_ws
source devel/setup.bash
roslaunch map_server map_server_sensor_updates.launch map_path:=/home/locobot/data/maps/cpsl_full.yaml
```

### 2. Running the navigation stack using lidar

```
roslaunch pcd2laserscan pcd2laserscan_lidar.launch use_rviz:=true
```

### 3. Running Lidar Navigation -UGV

terminal 1:

```
roslaunch xsens_mti_driver xsens_mti_node.launch
```

terminal 2:

```
roslaunch radar_launch kobuki.launch
```

terminal 3:
```
roslaunch velodyne_pointcloud VLP16_points.launch
```

terminal 4:
```
roslaunch map_server map_server_sensor_updates.launch map_path:=/home/locobot/data/maps/cpsl_full.yaml
```

terminal 5:
```
roslaunch odometry_ros lidar.launch
```

terminal 6:
```
roslaunch pcd2laserscan_lidar.launch use_rviz:=false
```

### 4. Running Radar Navigation

terminal 1:

```
roslaunch xsens_mti_driver xsens_mti_node.launch
```

terminal 2:

```
roslaunch radar_launch kobuki.launch
```

terminal 3: start front radar
```
#unplug IWR6843 (front radar)
ls /dev/ttyUSB* 
#plug IWR6843 back in
ls /dev/ttyUSB* #record the two new USB values
#go to /home/locobot/CPSL_TI_Radar/CPSL_TI_Radar/json_radar_settings/radar_6843.json and update the CLI_port and data_port (assenidng order) with the new USB values. Then save it
cd CPSL_TI_Radar/CPSL_TI_Radar
poetry run python run_radar.py --json_config radar_6843.json
```
terminal 4: start back radar
```
cd CPSL_TI_Radar/CPSL_TI_Radar
poetry run python run_radar.py --json_config radar_1443.json
```
terminal 5:
```
roslaunch map_server map_server_sensor_updates.launch map_path:=/home/locobot/data/maps/cpsl_full.yaml
```

terminal 6:
```
roslaunch odometry_ros radar.launch
```

terminal 7:
```
roslaunch pcd2laserscan pcd2laserscan_radar_nav.launch use_rviz:=false
```
terminal 8: to capture dataset of navigation trial
```
roslaunch radar_launch IWR6843_run_nav_dataset.launch config_file:=IWR_multi.json radar_enable:=true lidar_enable:=true camera_enable:=true imu_enable:=true vehicle_vel_enable:=true dataset_path:=/home/locobot/data/nav/wilk_basement_1 frame_rate:=20.0 frame_rate_high_speed_sensors:=200 views:=false
```

### 5. Running Radar Mapping

The generated maps aren't great with this, but the code runs

terminal 1:

```
roslaunch xsens_mti_driver xsens_mti_node.launch
```

terminal 2:

```
roslaunch radar_launch kobuki.launch
```

terminal 3: start front radar
```
#unplug IWR6843 (front radar)
ls /dev/ttyUSB* 
#plug IWR6843 back in
ls /dev/ttyUSB* #record the two new USB values
#go to /home/locobot/CPSL_TI_Radar/CPSL_TI_Radar/json_radar_settings/radar_6843.json and update the CLI_port and data_port (assenidng order) with the new USB values. Then save it
cd CPSL_TI_Radar/CPSL_TI_Radar
poetry run python run_radar.py --json_config radar_6843.json
```
terminal 4: start back radar
```
cd CPSL_TI_Radar/CPSL_TI_Radar
poetry run python run_radar.py --json_config radar_1443.json
```
terminal 6:
```
roslaunch odometry_ros radar.launch 
```

terminal 7:
```
roslaunch pcd2laserscan pcd2laserscan_radar_map.launch
```

Terminal 8:
```
roslaunch pcd2laserscan hector_mapping_revised.launch
```

### 6. Running Lidar Odometry -UAV


terminal 1:
```
roslaunch livox_ros_driver2 rviz_MID360.launch rviz_enable:=false
```

terminal 2:
```
roslaunch map_server map_server_sensor_updates.launch map_path:=/home/cpsl/data/maps/north.yaml
```

terminal 3:
```
roslaunch odometry_ros lidar_uav.launch
```

### 5. Athena Demo Visualization
In addition to running the pipeline, we have several viewer's available for visualizing the feed

USB camera:
```
roslaunch usb_cam usb_cam_cpsl.launch
```

RVIZ (run the "rviz" command in a new terminal for each viewer) configurations
* CPSL_ROS_Navigation_and_Mapping/pcd2laserscan/config/navigation.rviz
* CPSL_ROS_Navigation_and_Mapping/odometry_ros/rviz_configs/lidar.rviz
* CPSL_ROS_Navigation_and_Mapping/odometry_ros/rviz_configs/radar.rviz

