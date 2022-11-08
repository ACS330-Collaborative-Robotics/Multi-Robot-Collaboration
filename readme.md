# ACS330 Collaborative Robotics - Group 1

This repository contains:
- ROS Driver for Mover6
- Gazebo Simulation of System

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

## How to build

### Mover6 Dashboard with RViz

```
catkin_make clean
catkin_make rebuild_cache
catkin_make
catkin_make install
roslaunch cpr_robot CPRMover6.launch
```

### Gazebo simulation

WIP

## Fixes

**Gazebo not launching properly**

`htop` and kill `gzserver` 

**ROS hasnt been setup properly**

run `source devel/setup.bash`