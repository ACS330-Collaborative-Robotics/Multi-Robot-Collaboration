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

**CPR robot not conetting/working after running ./reconnect_robo.sh**
Probably dont have ifconfig
```
apt install net-tools
```

**Failed to launch joint_position_controller**

Need to install ros-control and ros-controllers using: `sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers`

**ROS Ignoring the first message on a topic**

ROS needs to know where each node is subscribed to and publishing to. This needs to happen before you can publish your first message. It then needs an amount of time to process this otherwise it will ignore the first message. e.g.

```python
## Incorrect method 
pub = rospy.Publisher("/topic_name", Joints, queue_size=10)
pub.publish(joints)

## More correct method 
pub = rospy.Publisher("/topic_name", Joints, queue_size=10)
# Delay or run calculations
pub.publish(joints)
```

**Gazebo Black Screen on Launch in WSL**

First, try install/update your graphics card drivers following [these instructions](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps#prerequisites).

If this doesnt fix the issues, or you have an Intel Xe Graphics card, you will need disable GPU acceleration, rendering only on your CPU. This will be very slow but better than nothing. Add `export  LIBGL_ALWAYS_SOFTWARE=1` to the end of your `.bashrc` file. Restart your terminal and it should work.
