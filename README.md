# localization_attack_ros
Localization attack virtual testbed in ROS (Noetic)+Gazebo

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
1.2 Run SLAM Node, in terminal 2:
```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```
1.3 Run Teleoperation Node, in terminal 3:
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
1.4 Save Map, in terminal 4:
```
rosrun map_server map_saver -f ~/map
```

### 2. Navigation in the simulated environment (Gazebo House)

2.1 Launch Simulation World, in terminal 1:
```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_house.launch
```

2.2 Run Navigation Node, in terminal 2:
```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```
Note that the map is obtained from Secction 1.4 above, and can be downloaded from this repo. Also, the path in map.yaml needs to be updated to reflect the local path.

2.3 Estimate Initial Pose

Initial Pose Estimation must be performed before running the Navigation as this process initializes the AMCL parameters that are critical in Navigation. TurtleBot3 has to be correctly located on the map with the LDS sensor data that neatly overlaps the displayed map.

Click the 2D Pose Estimate button in the RViz menu.

Click on the map where the actual robot is located and drag the large green arrow toward the direction where the robot is facing.
Repeat step 1 and 2 until the LDS sensor data is overlayed on the saved map.

2.4 Precisely locate the robot on the map (Optional)

2.4.1 Launch keyboard teleoperation node to precisely locate the robot on the map.
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
2.4.2 Move the robot back and forth a bit to collect the surrounding environment information and narrow down the estimated location of the TurtleBot3 on the map which is displayed with tiny green arrows.
 
2.4.3 Terminate the keyboard teleoperation node by entering Ctrl + C to the teleop node terminal in order to prevent different cmd_vel values are published from multiple nodes during Navigation.
