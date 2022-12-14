# ACS330 Multi-Robot Collaboration - Group 1

Efficient coordination of a multi-robot team is the key challenge in robotic application domains such as manufacturing, construction and service robotics. In cooperative manipulation tasks, the system dynamics result from the complex interaction of several manipulators handling a common object. However, the widespread use of robots in flexible factories  is limited by the robots’ inability to safely collaborate with one another. The robots’ incapacity to coordinate, communicate, and understand their actions, roles, and task statuses thus decreases the robots’ usefulness in applications where tasks cannot be completed by a single robot. This limitation is driven by both the absence of tools and protocols needed for describing collaborative functions.

This project shall look at the creation of a sophisticated model-based control algorithm for the effective and efficient interaction of multiple robots into their production processes.

# How to Build and Run

## ROS/Gazebo simulation build

1. Navigate to `~/catkin_ws` in Terminal and clone this GitHub repo with `git clone https://github.com/ACS330-Collaborative-Robotics/Gazebo_Sim.git`

2. Only the `src` and build scripts are stored in the GH Repo so now build the `build` and `devel` folder by running `./build_sim.sh`. **NOTE: `cpr-robot` package is not built with this script to reduce build time.** To build `cpr-robot`, delete `build` and `devel` and run `catkin_make`.

3. Each node is started separately, in its own Terminal tab to allow easier testing of individual nodes. This is **made much easier by using the Tabs feature** in your Terminal program. Each node (usually) has its own `.sh` script to start the node and can be stopped with `Ctrl+C`.

## Startup Instructions

Run each of the following in its own terminal tab, after running `cd ~/catkin_ws`.

 - `roscore` - ROS Core
 - `./run_sim.sh` - Gazebo Simulation, Sim Robot Joint Controller, Block Spawner
 - `./run_demo.sh` - Block Position Publisher, Inverse Kinematics, Kinematic Movement, Near Block Assignment Selection

### Mover6 Dashboard with RViz

```
catkin_make clean
catkin_make rebuild_cache
catkin_make
catkin_make install
roslaunch cpr_robot CPRMover6.launch
```

# ROS Information

Robot Namespaces - `robot_ns = ["mover6_a", "mover6_b]`

Robot Joints - `1 -> 6`

## Core nodes

| Nickname | Package | Description | Startup Script |
| - | - | - | - |
| `./run_sim.sh` |  |  |  |
| ROS Core | roscore | ROS Core required for ROS to Function | `roscore` |
| Gazebo Simulation | mover6_gazebo | Launches Gazebo and spawns a mover6 robot | `./run_sim.sh` |
| Sim Robot Joint Controller | mover6_control | Starts the listener node for Sim Robot Joint Positions | `./run_sim_control.sh` |
| Block Spawner | block_controller | Randomly generate a large number of blocks at random rotations in the workspace | `rosrun block_controller spawn_blocks.py` |
| `./run_demo.sh` |  |  |  |
| Block Position Publisher | block_pos_talker | Gathers block positions from gazebo and publishes them in `Blocks` message format to `/blocks_pos` | `rosrun block_controller block_pos_talker.py` |
| Inverse kinematics | inv_kinematics | Runs service `inverse-kinematics` and publish inverse kinematics to relevant joint position controller | `rosrun inv_kinematics inv_kin_srv.py` |
| Nearest Block Assignment Selection | assignment_selection | Finds which robot is closest to each robot and publishes to `robot_namespace/next_block` with 2 second cadence. | `rosrun assignment_selection block_selection.py` |
| Path Planner | path_planning | Mega node using OOP to plan and execute pick and place operations. | `rosrun path_planning path_plan.py`

## Additional nodes

| Nickname | Package | Description | Startup Script |
| - | - | - | - |
| Joint Position Movement Demo | movement_demo | Moves the mover6 joint's through the full range of motion via joint position | `rosrun mover6_joint_movement_demo joint_movement_demo.py`|
| Kinematics Movement Demo | movement_demo | Moves both mover6 robots to 5cm above randomly selected block, alternating robots on 2 second cadence. | `rosrun mover6_joint_movement_demo kinematics_movement_demo.py` |
| Old Inverse kinematics (Depreciated) | matlab_global_node_XXXXX | Listen on `/command_pos` for xyz coords and publish inverse kinematics to relevant joint position controller | `inv_kin_ros` in MATLAB |

## Topics

| Nickname | Name | Data Format | Python Data Format Import | Publishers | Subscribers |
| - | - | - | - | - | - |
| Block Positions | `/blocks_pos` | `block_controller Blocks` | `from block_controller.msg import Block, Blocks` | `block_controller block_pos_talker.py` | `assignment_selection block_selection.py` |
| Gazebo Model States (All Models) | `/gazebo/model_states` | `gazebo_msgs ModelStates` | `from gazebo_msgs.msg import ModelStates` | Gazebo | `block_controller block_pos_talker.py` | 
| Next block to pick | `robot_ns/next_block` | `std_msgs Strings` | `from std_msgs.msg import String` | `assignment_selection block_selection.py` | `movement_demo basic_kinematic_movement.py` |
| Gazebo Joint Position Controller | `robot_ns/jointX_position_controller/command` | `from std_msgs.msg import Float64` | `inv_kinematics inv_kin_srv.py`, `movement_demo joint_movement_demo.py`, `inv_kin_ros.m` | Gazebo |

## Services

| Nickname | Name | Location | Python Import | Input Format | Response Format |
| - | - | - | - | - | - |
| Specific Model Position | `gazebo/get_model_state'` | Gazebo | `from gazebo_msgs.srv import GetModelState` | `string model_name`, `string relative_entity_name` | `gazebo_msgs ModelState` |
| URDF Spawner | `gazebo/spawn_urdf_model` | Gazebo | `from gazebo_msgs.srv import SpawnModel` | `gazebo_msgs SpawnModel` | `bool success`, `string status_message` |
| ikpy Inverse Kinematics | `inverse_kinematics` | `inv_kinematics inv_kin_srv.py` | `from inv_kinematics.srv import InvKin` | `gazebo_msgs ModelState` | `bool success` |
| Path Planner | `path_planner` | `path_planning path_plan.py` | `from path_planning.srv import PathPlan` | `string robot-name`, `geometry_msg/Pose end_pos`, `string block_name` | `bool success` |

# Run on Windows through Windows Subsystems for Linux

The project can be run on Windows using WSL2 based on [this guide from DR Tom Howard TuoS](https://github.com/tom-howard/COM2009/wiki/Working-On-Your-Own-Computer#installing-on-windows-using-wsl)

## Ubuntu (Linux) Install

Windows 11 works best but can using Windows 10 with more effort.

Install Windows Subsystems for Linux 2 using Ubuntu 20.04 Distribution with: `wsl --install -d Ubuntu-20.04` in Terminal. Use Windows Terminal, not command prompt! You may need to install Windows Terminal [here](https://apps.microsoft.com/store/detail/windows-terminal/9N0DX20HK701?hl=en-gb&gl=gb).


Set a short username i.e. your first name and a password you can remember and type quickly - it doesn't need to be massively secure!

Reboot computer.

Type `wsl --status` and make sure it returns:
```bash
Default Distribution: Ubuntu-20.04
Default Version: 2
```

Now we're gonna get to the Ubuntu command line and install ROS. 

Go to Terminal and click the down arrow in the top tab bar, then click `Ubuntu 20.04`. You should have a new terminal tab which says `Username@PC_Name`, this is the Ubuntu terminal.

## ROS Install

Now, install ROS by running each of the following commands:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
```

Setup terminal to run ROS commands properly:
```bash
cd
pico .bashrc # Or open this text file in a text editor
# Add following to the end of the file
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

Close the terminal and reopen it.

## Dependencies Install

```bash
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
sudo apt python3-pip
pip install ikpy
pip install numpy --upgrade
```

## Clone repository

```bash
mkdir ~/catkin_ws
cd ~/catkin_ws
git clone https://github.com/ACS330-Collaborative-Robotics/Multi-Robot-Collaboration.git .
```

## Open in Editor

Use VSCode which can be downloaded [here](https://code.visualstudio.com/download).

To open run `code .` in Ubuntu terminal in `~/catkin_ws`

## Setup GitHub credentials

Log into GitHub through VSCode then setup username and email using:

```bash
git config --global user.name "USERNAME"
git config --global user.email "EMAIL"
```

## Run Software

See *How to Build* above.

# Useful Links

- [CPR Robots Driver Repo](https://github.com/CPR-Robots/cpr_robot)
- [ROS Wiki](http://wiki.ros.org/Documentation)
- CPR Robots Docs - /src/cpr-robot/doc/html/index.html
- [UR5 ROS Gazebo](https://github.com/lihuang3/ur5_ROS-Gazebo)
- [Gazebo Message Docs](http://docs.ros.org/en/noetic/api/gazebo_msgs/html/index-msg.html)

## Gazebo Tutorials

- [ROS integration overview](https://classic.gazebosim.org/tutorials?tut=ros_overview) 
- [Using roslaunch](https://classic.gazebosim.org/tutorials?tut=ros_roslaunch&cat=connect_ros)
- [URDF in Gazebo](https://classic.gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros)
- [ROS plugin](https://classic.gazebosim.org/tutorials?tut=ros_plugins&cat=connect_ros)
- [Convert xacro to URDF](https://www.oreilly.com/library/view/mastering-ros-for/9781788478953/d04a8d45-b84b-4c3e-ad03-eb158fe5f451.xhtml)
- [ROS Control](https://classic.gazebosim.org/tutorials?tut=ros_control)
- [Creating and Spawning Custom URDF Objects in Simulation](http://wiki.ros.org/simulator_gazebo/Tutorials/SpawningObjectInSimulation)
- [Manipulate and interact with simulation and simulated objects](http://wiki.ros.org/simulator_gazebo/Tutorials/Gazebo_ROS_API)

# Fixes

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
