# Environment Setup
## Installation
```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install ros-kinetic-uol-cmp3103m
```

Needed if there are conflicting packages:
```
sudo apt-get purge "*gazebo*"
```
## Setup

```
mkdir -p ~/catkin_ws/src; cd ~/catkin_ws/src; catkin_init_workspace .;
catkin_create_pkg assignment rospy std_msgs geometry_msgs
mkdir ~/catkin_ws/src/assignment/scripts
cd ~/catkin_ws
catkin_make
```

```
source ~/catkin_ws/devel/setup.bash 
roscore
```

```
source ~/catkin_ws/devel/setup.bash
roslaunch uol_turtlebot_simulator object-search-training.launch
```

```
source ~/catkin_ws/devel/setup.bash
spyder
```

```
rosrun rviz rviz
```

```
rosrun rqt_graph rqt_graph
```
