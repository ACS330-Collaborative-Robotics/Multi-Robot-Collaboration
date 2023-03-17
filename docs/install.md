# How to Build and Run

## ROS/Gazebo simulation build

1. Navigate to `~/catkin_ws` in Ubuntu Terminal.

2. Build the project by running `catkin_make`. Only the `src` and build scripts are stored in the GH Repo so this will build the `build` and `devel` folders.

3. Each set of nodes is started separately, in its own Terminal tab to allow easier testing of individual nodes. This is **made much easier by using the Tabs feature** in your Terminal program. Each group of nodes has its own `.sh` script to start the node and can be stopped with `Ctrl+C`.

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

# Connecting and setting up Networking

## Connecting Linux machine to robotwlan

```
nmcli c show
nmcli c down eduroam
nmcli c up robotwlan
```

You will need to conect your device to the `robotwlan` network, accessable in certain areas of the university or the wired university network.

## Setup localhost after network

```
$ export ROS_HOSTNAME=localhost
$ export ROS_MASTER_URI=http://localhost:11311
```

## Roscore Host Mechine

Run to setup roscore on network.

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

Run in every tab opened afterwards.

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

Next, setup terminal to run ROS commands properly as outlined below. `pico` opens a Command Line text editor which will allow you to edit the `.bashrc` files and add the relevant lines. The pico editor is navigated with the arrow keys and you can type as normal. To save press `Ctrl+X` then `Enter`.
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
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-apriltag ros-noetic-apriltag-ros ros-noetic-image-pipeline ros-noetic-usb-cam ros-noetic-trac-ik
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

# Run Software

To run Gazebo simulation use './run_sim.sh'.

To run a demonstration of the arm behaviour after running the simulation, use './run_demo.sh'.


## Reccomended VS Code Extension

There are a few VSCode exentension that will make your life much easier so I would reccomend installing the ones listed below using the Extensions tab on the right.
 - WSL - This extension makes VSCode play nicely with the WSL system. This will make the terminal in VSCode into an Ubuntu terminal too which makes running commands much easier.
 - GitLens - Makes VSCode and GitHub play nicely together.
 - C/C++
 - Python
 - Pylance
 - Todo Tree
 - A nice theme - I reccomend One Dark Pro ;)
