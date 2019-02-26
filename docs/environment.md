# Environment Setup

```
sudo apt-get update
sudo apt-get upgrade
```

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