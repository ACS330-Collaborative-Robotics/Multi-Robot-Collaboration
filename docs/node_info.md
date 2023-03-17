# Information about Nodes Topics Services and Actions

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
| Joint Controller | joint_controller | Controls whether or not simulation and physical robots recieve commands as required. Runs once per robot. Needs lauch file. | `rosrun joint_controller joint_controller.py` |
| Gripper Controller | joint_controller | Controlls the Gripper based on the selection channel. Runs once per robot. Needs lauch file. | `rosrun joint_controller gripper_controller.py` |
| Nearest Block Assignment Selection | assignment_selection | Finds which robot is closest to each robot and publishes to `robot_namespace/next_block` with 2 second cadence. | `rosrun assignment_selection block_selection.py` |
| Path Planner | path_planning | Mega node using OOP to plan and execute pick and place operations. | `rosrun path_planning path_plan.py` |
| Block Fiducial Detector | fiducial_recognition | Runs camera setup, image processing, apriltag detection and cartesian block coordinates relative to mover6a | `roslaunch fiducial_recognition fiducial_recognition.launch camera_name:=/usb_cam image_topic:=image_rect_color` |
| `./connect_robo_setup.sh` |  | Runs the comands to connect the robot arms to the computer. Needs to be run once each time the computer boots. |   |
| `./connect_mover6a.sh` `./connect_mover6b.sh` | | | Use launch files so that they are namespaced correctly. |
| CPR robot controller | cpr_robot | Modifed version of the controller provided in the CPR_Robot git page. The modifications allow for they system to be namespaced. | |
| Mover6 driver | joint_controller| Takes the output form the joint controller and passes it to rvis and the CPR_Robot package. | |

## Additional nodes

| Nickname | Package | Description | Startup Script |
| - | - | - | - |
| Joint Position Movement Demo | movement_demo | Moves the mover6 joint's through the full range of motion via joint position | `rosrun mover6_joint_movement_demo joint_movement_demo.py`|
| Kinematics Movement Demo | movement_demo | Moves both mover6 robots to 5cm above randomly selected block, alternating robots on 2 second cadence. | `rosrun mover6_joint_movement_demo kinematics_movement_demo.py` |
| Test Kinematic Chain | movement_demo | Executes Path Plan service call once | `rosrun mover6_joint_movement_demo test_kinematic_chain.py` |
| Mover6 Driver | joint_controller | Relays data from physical robot demand position to physical robot including change of units. |`export ROS_NAMESPACE=/mover6_a_p` `rosrun joint_controller mover6_driver` |
| Fixed Zone Controller | zone_controller | Publishes a pair of fixed zones for testing purposes. | `rosrun zone_controller fixed_zone.py` |
| Zone Point Detection Demo (Including TF Forward Kinematics demo) | zone_detection | Checks whether a point is in each published zone. Also demonstrate forward kinematics using Transform Trees. | `rosrun zone_detection point_detect.py` |

## Topics

| Nickname | Name | Data Format | Python Data Format Import | Publishers | Subscribers |
| - | - | - | - | - | - |
| Roscore | `/rosout` `/rosout_agg` | Roscore setup nodes |  |  |  |
| Block Positions | `/blocks_pos` | `block_controller Blocks OR tf2_listener` | `from block_controller.msg import Block, Blocks` | `block_controller block_pos_talker.py OR tf2_listener.py` | `assignment_selection block_selection.py` |
| Gazebo Model States (All Models) | `/gazebo/model_states` | `gazebo_msgs ModelStates` | `from gazebo_msgs.msg import ModelStates` | Gazebo | `block_controller block_pos_talker.py` | 
| Next block to pick | `robot_ns/next_block` | `std_msgs Strings` | `from std_msgs.msg import String` | `assignment_selection block_selection.py` | `movement_demo basic_kinematic_movement.py` |
| Gazebo Joint Position Controller | `robot_ns/jointX_position_controller/command` | `from std_msgs.msg import Float64` | `inv_kinematics inv_kin_srv.py`, `movement_demo joint_movement_demo.py`, `inv_kin_ros.m` | Gazebo |
| Setup/RVis | `/clicked_point`, `/initialpose`, `/move_base_simple/goal` | Setup and run by rvis/CPR_Robot driver | We do not use however are needed for setup |  |  |
| Mover6 Input Channel | `/robot_ns_p/InputChannels` | cpr_robot/ChannelStates |  | /mover6_a_p/CPRMover6 | /mover6_a_p/rviz |
| Mover6 Output Channel | `/robot_ns_p/OutputChannels` | cpr_robot/ChannelStates |  | /mover6_a_p/CPRMover6 | /mover6_a_p/rviz |
| Mover6 Move Commands | `/robot_ns_p/JointJog` | control_msgs/JointJog |  | /mover6_a_p/mover6_driver /mover6_a_p/rviz | /mover6_a_p/CPRMover6 |
| Current Joint Angles | `/robot_ns_p/joint_states` | sensor_msgs/JointState |  | /mover6_a_p/CPRMover6 | /mover6_a_p/robot_state_publisher /mover6_a_p/mover6_driver /mover6_a_p/rviz |
| Desired Joint Angles | `/robot_ns_p/physical/joint_angles` | custom_msgs/Joints |  | /mover6_a_p/mover6_driver |  |
| Current Moving State | `/robot_ns_p/physical/moving_state` | std_msgs/String |  | /mover6_a_p/mover6_driver |  |
| CPR Robot State | `/robot_ns_p/robot_state` | cpr_robot/RobotState |  | /mover6_a_p/CPRMover6 | /mover6_a_p/rviz |
| Mover6 gripper Controller | /robot_ns_p/gripper_state | std_msgs/Bool | `from std_msgs.msg import Bool` |  | /mover6_a_p/gripper_controller |

## Services

| Nickname | Name | Location | Python Import | Input Format | Response Format |
| - | - | - | - | - | - |
| Specific Model Position | `gazebo/get_model_state'` | Gazebo | `from gazebo_msgs.srv import GetModelState` | `string model_name`, `string relative_entity_name` | `gazebo_msgs ModelState` |
| URDF Spawner | `gazebo/spawn_urdf_model` | Gazebo | `from gazebo_msgs.srv import SpawnModel` | `gazebo_msgs SpawnModel` | `bool success`, `string status_message` |
| ikpy Inverse Kinematics | `inverse_kinematics` | `inv_kinematics inv_kin_srv.py` | `from inv_kinematics.srv import InvKin` | `gazebo_msgs ModelState` | `bool success` |
| Path Planner | `path_planner` | `path_planning path_plan.py` | `from path_planning.srv import PathPlan` | `string robot-name`, `geometry_msg/Pose end_pos`, `string block_name` | `bool success` |
