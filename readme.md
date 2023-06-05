# ACS330 Multi-Robot Collaboration - Group 1

Efficient coordination of a multi-robot team is the key challenge in robotic application domains such as manufacturing, construction and service robotics. In cooperative manipulation tasks, the system dynamics result from the complex interaction of several manipulators handling a common object. However, the widespread use of robots in flexible factories  is limited by the robots’ inability to safely collaborate with one another. The robots’ incapacity to coordinate, communicate, and understand their actions, roles, and task statuses thus decreases the robots’ usefulness in applications where tasks cannot be completed by a single robot. This limitation is driven by both the absence of tools and protocols needed for describing collaborative functions.

This project shall look at the creation of a sophisticated model-based control algorithm for the effective and efficient interaction of multiple robots into their production processes.

# ROS Information

Robot Namespaces - `robot_ns = ["mover6_a", "mover6_b]`

Robot Joints - `1 -> 6`

**Node Graph Diagram**

![ROS Node Graph](docs/node_graph.png)

**Path Planner Class Diagram**

![Path Planner Class Diagram](docs/path_planner_class_diagram.png)

## Core nodes

| Nickname | Package | Description |
| - | - | - |
| `./run_sim.sh` |  |  |
| Gazebo Simulation | mover6_gazebo | Launches Gazebo and spawns robots. |
| Simulation Robot Joint Controller | mover6_control | Starts the listener node for simulation robot joint controller. |
| Block Spawner | block_controller | Spawns a number of blocks at random rotations in the workspace within the robot workspace. |
| `./run_demo.sh` |  |  |
| Block Position Publisher | block_pos_talker | Gathers block positions from gazebo and publishes them in `Blocks` message format to `/blocks_pos`. |
| Inverse kinematics | inv_kinematics | Runs service `inverse-kinematics` and publish inverse kinematics to relevant joint position controller. |
| Joint Controller | joint_controller | Controls whether or not simulation and physical robots recieve commands as required. Runs once per robot. |
| Gripper Controller | joint_controller | Controls the Gripper based on the selection channel. Runs once per robot. |
| Assignment Selection | assignment_selection | Decides robot assignments and calls Path Planner actions. |
| Path Planner | path_planning | Mega node using OOP to plan and execute pick and place operations. |
| Block Fiducial Detector | fiducial_recognition | Runs camera setup, image processing, apriltag detection and cartesian block coordinates relative to mover6a. |
| `./connect_robo_setup.sh` |  | Runs the commands to connect the robot arms to the computer. Needs to be run once each time the computer boots. |
| `./connect_mover6a.sh` `./connect_mover6b.sh` | | Use launch files so that they are namespaced correctly. |
| CPR robot controller | cpr_robot | Modified version of the controller provided in the CPR_Robot git page. The modifications allow for controller to be namespaced. |
| Mover6 driver | joint_controller| Takes the output from the joint controller and passes it to rvis and the CPR_Robot package. |

## Additional nodes

| Nickname | Package | Description | Startup Script |
| - | - | - | - |
| Physical Robot homing | movement_demo | Takes the physical robot to the same starting point as the simulation robot | `rosrun movement_demo physical_robot_homing.py`  |
| Joint Sweep Test | movement_demo | Moves the mover6 joint's through the full range of motion via joint position | `rosrun movement_demo joint_sweep_test.py`|
| Inverse Kinematics Test | movement_demo | Completes one inverse kinematics service call. | `rosrun movement_demo inverse_kinematic_test.py` |
| Joint Behaviour Test | movement_demo | Moves the a mover6 joint to a specific position and plots the simulation and physical response. | `rosrun movement_demo joint_behaviour_test.py`|
| Path Planner Test | movement_demo | Executes Path Plan action once | `rosrun movement_demo path_planner_test.py` |
| Fixed Zone Controller | zone_controller | Publishes a pair of fixed zones for testing purposes. | `rosrun zone_controller fixed_zone.py` |
| Zone Point Detection Demo | zone_detection | Checks whether a point is in each published zone. Also demonstrate forward kinematics using Transform Trees. | `rosrun zone_detection point_detect.py` |

## Key Topics

| Nickname | Name | Data Format |
| - | - | - |
| Block Positions | `blocks_pos` | block_controller/Blocks |
| Gazebo Joint Position Controller | `robot_ns/jointX_position_controller/command` | std_msgs/Float64 |
| Mover6 Input Channel | `robot_ns_p/InputChannels` | cpr_robot/ChannelStates |
| Mover6 Output Channel | `robot_ns_p/OutputChannels` | cpr_robot/ChannelStates |
| Mover6 Move Commands | `robot_ns_p/JointJog` | control_msgs/JointJog |
| Current Joint Angles | `robot_ns_p/joint_states` | sensor_msgs/JointState |
| Desired Joint Angles | `robot_ns_p/physical/joint_angles` | custom_msgs/Joints |
| Current Moving State | `robot_ns_p/physical/moving_state` | std_msgs/String |
| CPR Robot State | `robot_ns_p/robot_state` | cpr_robot/RobotState |
| Mover6 Gripper Control | `robot_ns/gripper_state` | std_msgs/Bool |

# How to Build and Run

## ROS/Gazebo simulation build

1. Navigate to `~/catkin_ws` in Ubuntu Terminal.

2. Build the project by running `catkin_make`. Only the `src` and build scripts are stored in the GH Repo so this will build the `build` and `devel` folders.

3. Each set of nodes is started separately, in its own Terminal tab to allow easier testing of individual nodes. This is **made much easier by using the Tabs feature** in your Terminal program. Each group of nodes has its own `.sh` script to start the node and can be stopped with `Ctrl+C`.

## Startup Instructions

### Sim Only Setup

Run each of the following in its own terminal tab, after running `cd ~/catkin_ws`.

 - `roscore` - ROS Core
 - `./run_sim.sh` - Gazebo Simulation, Sim Robot Joint Controller, Block Spawner
 - `./run_demo.sh` - Block Position Publisher, Inverse Kinematics, Kinematic Movement, Near Block Assignment Selection

 ### Full System Setup

 Run each of the following in its individual terminal tab, after running `cd ~/catkin_ws`. In each terminal tab make sure it is connected to the network.

Main Machine
 - `roscore` - ROS Core

Raspberry Pis
 - `./connect_robo_setup.sh` - Only need to run this if booting up the pis not restarting the system
 - `./connect_mover6X.sh` - Connects to the Mover6 to the network

Main Machine
 - `./run_sim_phs.sh` - Gazebo Simulation, Sim Robot Joint Controller, Block Spawner
 - `./clear_workspace.sh` - sets robots to a know position

Camera Machine
 - `./fad_sensor_start.sh` - This runs the camera for the system

Safety Stop Machine
 - `./lidar.sh` - This connects to the lidar sensor
 - `./safety_stop_manual.sh` - This runs the manual safety stop that can be run on any machine

Main Machine
 - `./open_cam_fad.sh` - This allows the operator of the main machine to see the fiducials
 - `./open_overhead_cam.sh` - This allows the operator of the main machine to see the workspace
 - `./run_demo.sh` - Block Position Publisher, Inverse Kinematics, Kinematic Movement, Near Block Assignment Selection

GUI Machine
  - `./gui.sh`



### Launching the Lidar

Run in terminal:
`sudo chmod 666 /dev/ttyUSB0` (worked on laptop)
OR:
`sudo chmod 777 /dev/ttyUSB0` (worked on Ubuntu)
Note: LIDAR does not appear to work through an extension cable

Then:
`roslaunch rplidar_ros rplidar.launch`

In a seperate terminal run the line below to publish the data from the lidar to /emergency_stop:
`rosrun human_zone human_detection.py`

### Alternative Startup Instructions

Useful for testing various systems using test scripts in `movement_demo`.

 - `roscore` - ROS Core
 - `./run_sim.sh` - Gazebo Simulation, Sim Robot Joint Controller, Block Spawner
 - `roslaunch movement_demo kinematics_nodes.launch` - Block Position Publisher, Inverse Kinematics, Joint Controller, Gripper Controller, Path Planner
 - `rosrun movement_demo XXXXX.py`

Where `XXXXX` is the desired test script.

### Mover6 Dashboard with RViz

```
catkin_make clean
catkin_make rebuild_cache
catkin_make
catkin_make install
roslaunch cpr_robot CPRMover6.launch
```

# Connecting and setting up Networking

## Connecting Linux machine to robotwlan

```
nmcli c show
nmcli c down eduroam
nmcli c up robotwlan
```

You will need to connect your device to the `robotwlan` network, accessible in certain areas of the university or the wired university network.

## Setup localhost after network

```
$ export ROS_HOSTNAME=localhost
$ export ROS_MASTER_URI=http://localhost:11311
```

## Roscore Host Machine

Run to set up the roscore on the network.

```
$ hostname -I
0.0.0.1         #Example ip
$ export ROS_HOSTNAME=0.0.0.1
$ export ROS_MASTER_URI=http://0.0.0.1:11311/
$ roscore
...
ROS_MASTER_URI=http://0.0.0.1:11311/       #Makesure you see this line in roscore output
...
```

Run in every tab opened afterward.

```
$ export ROS_HOSTNAME=0.0.0.1
$ export ROS_MASTER_URI=http://0.0.0.1:11311/
```

## Worker Machine

```
$ hostname -I
0.0.0.2         #Example ip
$ export ROS_HOSTNAME=0.0.0.2
$ export ROS_MASTER_URI=http://0.0.0.1:11311/
```

Run in every tab you open.

```
$ export ROS_HOSTNAME=0.0.0.2
$ export ROS_MASTER_URI=http://0.0.0.1:11311/
```

# Run on Windows through Windows Subsystems for Linux

The project can be run on Windows using WSL2 based on [this guide from DR Tom Howard TuoS](https://github.com/tom-howard/COM2009/wiki/Working-On-Your-Own-Computer#installing-on-windows-using-wsl)

## Ubuntu (Linux) Install

Windows 11 works best but can use Windows 10 with additional software.

Install Windows Subsystems for Linux 2 using Ubuntu 20.04 Distribution with: `wsl --install -d Ubuntu-20.04` in Terminal. Use Windows Terminal, not command prompt! You may need to install Windows Terminal [here](https://apps.microsoft.com/store/detail/windows-terminal/9N0DX20HK701?hl=en-gb&gl=gb).

Set a short username i.e. your first name and a password you can remember and type quickly - it doesn't need to be massively secure!

Reboot computer.

Type `wsl --status` and make sure it returns:
```bash
Default Distribution: Ubuntu-20.04
Default Version: 2
```

Now we're gonna get to the Ubuntu command line and install ROS. 

Go to Terminal and click the down arrow in the top tab bar, then click `Ubuntu 20.04`. You should have a new terminal tab displaying `Username@PC_Name`, this is the Ubuntu terminal.

## ROS Install

Now, install ROS by running each of the following commands:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
```

Next, set up your terminal to run ROS commands properly as outlined below. `pico` opens a Command Line text editor which will allow you to edit the `.bashrc` files and add the relevant lines. The pico editor is navigated with the arrow keys and you can type as normal. To save press `Ctrl+X` then `Enter`.

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
sudo apt-get update
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-apriltag ros-noetic-apriltag-ros ros-noetic-image-pipeline ros-noetic-usb-cam ros-noetic-trac-ik python3-pil.imagetk ros-noetic-rplidar-ros
sudo apt install python3-pip
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

## Recommended VS Code Extension

There are a few VSCode extensions that will make your life much easier so I would recommend installing the ones listed below using the Extensions tab on the right.
 - WSL - This extension makes VSCode play nicely with the WSL system. This will make the terminal in VSCode into an Ubuntu terminal too which makes running commands much easier.
 - GitLens - Makes VSCode and GitHub play nicely together.
 - C/C++
 - Python
 - Pylance
 - Todo Tree
 - A nice theme - I recommend One Dark Pro ;)

## Run Software

See *How to Build* above.

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

**CPR robot not connecting/working after running ./reconnect_robo.sh**

Probably don't have `ifconfig` which can be installed with: `sudo apt install net-tools`.

**Failed to launch joint_position_controller**

Need to install ros-control and ros-controllers using: `sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers`

**ROS Ignoring the first message on a topic**

ROS needs to know where each node is subscribed to and publishing too. This needs to happen before you can publish your first message. It then needs an amount of time to process this otherwise it will ignore the first message. e.g.

```python
## Incorrect method ##
pub = rospy.Publisher("/topic_name", Joints, queue_size=10)
pub.publish(joints)

## More correct method ##
pub = rospy.Publisher("/topic_name", Joints, queue_size=10)
# Delay or run calculations
pub.publish(joints)
```

**Gazebo Black Screen on Launch in WSL**

First, try to install or update your graphics card drivers following [these instructions](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps#prerequisites).

If this doesn't fix the issues, or you have an Intel Xe Graphics card, you will need to disable GPU acceleration, rendering only on your CPU. This will be very slow but better than nothing. Add `export  LIBGL_ALWAYS_SOFTWARE=1` to the end of your `.bashrc` file. Restart your terminal and it should work.

**apriltag_ros missing a camera calibration file**

In Google Drive in the `Technical Documentation/camera calibration` folder, there is a required calibration file. `head_camera.yaml` must be placed in a folder called `camera_info` in `home/.ros`.

**roslaunch not finding packages that exist**

run - `source /home/uos/catkin_ws/devel/setup.bash` (or other path).

**Logitech C920 Parameters**

`v4l2-ctl -d /dev/video0 --list-ctrls`

```
                     brightness 0x00980900 (int)    : min=0 max=255 step=1 default=128 value=128
                       contrast 0x00980901 (int)    : min=0 max=255 step=1 default=128 value=128
                     saturation 0x00980902 (int)    : min=0 max=255 step=1 default=128 value=0
 white_balance_temperature_auto 0x0098090c (bool)   : default=1 value=1
                           gain 0x00980913 (int)    : min=0 max=255 step=1 default=0 value=255
           power_line_frequency 0x00980918 (menu)   : min=0 max=2 default=2 value=2
      white_balance_temperature 0x0098091a (int)    : min=2000 max=6500 step=1 default=4000 value=2911 flags=inactive
                      sharpness 0x0098091b (int)    : min=0 max=255 step=1 default=128 value=128
         backlight_compensation 0x0098091c (int)    : min=0 max=1 step=1 default=0 value=0
                  exposure_auto 0x009a0901 (menu)   : min=0 max=3 default=3 value=3
              exposure_absolute 0x009a0902 (int)    : min=3 max=2047 step=1 default=250 value=333 flags=inactive
         exposure_auto_priority 0x009a0903 (bool)   : default=0 value=1
                   pan_absolute 0x009a0908 (int)    : min=-36000 max=36000 step=3600 default=0 value=0
                  tilt_absolute 0x009a0909 (int)    : min=-36000 max=36000 step=3600 default=0 value=0
                 focus_absolute 0x009a090a (int)    : min=0 max=250 step=5 default=0 value=0
                     focus_auto 0x009a090c (bool)   : default=1 value=0
                  zoom_absolute 0x009a090d (int)    : min=100 max=500 step=1 default=100 value=100
```
