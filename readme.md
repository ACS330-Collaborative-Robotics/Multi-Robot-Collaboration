# ACS330 Collaborative Robotics - Group 1

This repository contains:
- ROS Driver for Mover6
- Gazebo Simulation of System

## How to build

### Gazebo simulation

1. Navigate to `~/catkin_ws` in Terminal and clone this GitHub repo with `git clone https://github.com/ACS330-Collaborative-Robotics/Gazebo_Sim.git`

2. Only the `src` and build scripts are stored in the GH Repo so now build the `build` and `devel` folder by running `./build_sim.sh`. **NOTE: `cpr-robot` package is not built with this script to reduce build time.** To build `cpr-robot`, delete `build` and `devel` and run `catkin_make`.

3. Each node is started separately, in its own Terminal tab to allow easier testing of individual nodes. This is **made much easier by using the Tabs feature** in your Terminal program. Each node (usually) has its own `.sh` script to start the node and can be stopped with `Ctrl+C`.

#### Core nodes (In initial startup order)

| Nickname | Package | Description | Startup Script |
| - | - | - | - |
| ROS Core | roscore | ROS Core required for ROS to Function | `roscore` |
| Gazebo Simulation | mover6_gazebo | Launches Gazebo and spawns a mover6 robot | `./run_sim.sh` |
| Sim Robot Joint Controller | mover6_control | Starts the listener node for Sim Robot Joint Positions | `./run_sim_control.sh` |
| Block Spawner | | To be run on startup, see  Block Spawner below.| |
| Block Position Publisher | block_pos_talker | Gathers block positions from gazebo and publishes them in `Blocks` message format to `/blocks_pos` | `rosrun block_controller block_pos_talker.py` |
| Inverse kinematics | inverse_kinematics | Runs service `inverse-kinematics` and publish inverse kinematics to relevant joint position controller | `rosrun inv_kinematics inv_kin_srv.py` |
| Kinematic Movement | movement_demo | Listens on `robot_namespace/next_block` for block names to move to, then calls inverse kinematic movement. | `rosrun movement_demo kinematic_movement.py` | 
| Nearest Block Assignment Selection | assignment_selection | Finds which robot is closest to each robot and publishes to `robot_namespace/next_block` with 2 second cadence. | `rosrun assignment_selection block_selection.py` |

#### Additional nodes

| Nickname | Package | Description | Startup Script |
| - | - | - | - |
| Block Spawner | block_controller | Randomly generate a large number of blocks at random rotations in the workspace | `rosrun block_controller spawn_blocks.py` |
| Joint Position Movement Demo | movement_demo | Moves the mover6 joint's through the full range of motion via joint position | `rosrun mover6_joint_movement_demo joint_movement_demo.py`|
| Kinematics Movement Demo | movement_demo | Moves both mover6 robots to 5cm above randomly selected block, alternating robots on 2 second cadence. | `rosrun mover6_joint_movement_demo kinematics_movement_demo.py` |
| Old Inverse kinematics (MATLAB) | matlab_global_node_XXXXX | Listen on `/command_pos` for xyz coords and publish inverse kinematics to relevant joint position controller | `inv_kin_ros` in MATLAB |

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
- [UR5 ROS Gazebo](https://github.com/lihuang3/ur5_ROS-Gazebo)
- [Gazebo Message Docs](http://docs.ros.org/en/noetic/api/gazebo_msgs/html/index-msg.html)

### Gazebo Tutorials

- [ROS integration overview](https://classic.gazebosim.org/tutorials?tut=ros_overview) 
- [Using roslaunch](https://classic.gazebosim.org/tutorials?tut=ros_roslaunch&cat=connect_ros)
- [URDF in Gazebo](https://classic.gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros)
- [ROS plugin](https://classic.gazebosim.org/tutorials?tut=ros_plugins&cat=connect_ros)
- [Convert xacro to URDF](https://www.oreilly.com/library/view/mastering-ros-for/9781788478953/d04a8d45-b84b-4c3e-ad03-eb158fe5f451.xhtml)
- [ROS Control](https://classic.gazebosim.org/tutorials?tut=ros_control)
- [Creating and Spawning Custom URDF Objects in Simulation](http://wiki.ros.org/simulator_gazebo/Tutorials/SpawningObjectInSimulation)
- [Manipulate and interact with simulation and simulated objects](http://wiki.ros.org/simulator_gazebo/Tutorials/Gazebo_ROS_API)

## Fixes

**Gazebo not launching properly**

`htop` and kill `gzserver` 

**Setup not done properly**

```bash
cd
pico .bashrc # Or open this text file in a text editor
# Add following to the end of the file
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

**Failed to launch joint_position_controller**

Need to install ros-control and ros-controllers using: `sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers`

**MATLAB not setup for ROS**

Requires `sudo apt install python3.9 python3.9-venv` and path set in MATLAB in `preferences>ROS Toolbox>Open ROS Toolbox Prefences`.
