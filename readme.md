# ACS330 Collaborative Robotics - Group 1

This repository contains:
- ROS Driver for Mover6
- Gazebo Simulation of System

## How to build

### Gazebo simulation

1. Navigate to `~/catkin_ws` in Terminal and clone this GitHub repo with `git clone https://github.com/ACS330-Collaborative-Robotics/Gazebo_Sim.git`

2. Only the `src` and build scripts are stored in the GH Repo so now build the `build` and `devel` folder by running `./build_sim.sh`. **NOTE: `cpr-robot` package is not built with this script to reduce build time.** To build `cpr-robot`, delete `build` and `devel` and run `catkin_make`.

3. Each node is started separately, in its own Terminal tab to allow easier testing of individual nodes. This is **made much easier by using the Tabs feature** in your Terminal program. Each node (usually) has its own `.sh` script to start the node and can be stopped with `Ctrl+C`.

| Nickname | Node Name(s) | Description | Startup Script |
| - | - | - | - |
| ROS Core | roscore | ROS Core required for ROS to Function | `roscore` |
| Gazebo Simulation | gazebo, gazebo_gui, spawn_urdf | Launches Gazebo and spawns a mover6 robot | `./run_sim.sh` |
| Sim Robot Joint Controller | robot_state_publisher, mover6/controller_spawner | Starts the listener node for Sim Robot Joint Positions | `./run_sim_control.sh` |
| Joint Position Movement Demo | joint_movement_demo | Moves the mover6 joint's through the full range of motion via joint position | `rosrun mover6_joint_movement_demo joint_movement_demo.py`|


### Mover6 Dashboard with RViz

```
catkin_make clean
catkin_make rebuild_cache
catkin_make
catkin_make install
roslaunch cpr_robot CPRMover6.launch
```

## Useful Links

- [CPR Robots Driver Repo](https://github.com/CPR-Robots/cpr_robot)
- [ROS Wiki](http://wiki.ros.org/Documentation)
- CPR Robots Docs - /src/cpr-robot/doc/html/index.html

### Gazebo Tutorials

- [ROS integration overview](https://classic.gazebosim.org/tutorials?tut=ros_overview) 
- [Using roslaunch](https://classic.gazebosim.org/tutorials?tut=ros_roslaunch&cat=connect_ros)
- [URDF in Gazebo](https://classic.gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros)
- [ROS plugin](https://classic.gazebosim.org/tutorials?tut=ros_plugins&cat=connect_ros)
- [Convert xacro to URDF](https://www.oreilly.com/library/view/mastering-ros-for/9781788478953/d04a8d45-b84b-4c3e-ad03-eb158fe5f451.xhtml)
- [ROS Control](https://classic.gazebosim.org/tutorials?tut=ros_control)

## Fixes

**Gazebo not launching properly**

`htop` and kill `gzserver` 

**ROS hasnt been setup properly**

run `source devel/setup.bash`

**Setup not done properly**

```bash
cd
pico .bashrc # Or open this text file in a text editor
# Add following to the end of the file
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```