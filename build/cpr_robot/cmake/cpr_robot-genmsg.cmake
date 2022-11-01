# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "cpr_robot: 2 messages, 3 services")

set(MSG_I_FLAGS "-Icpr_robot:/home/conor/catkin_ws/src/cpr_robot/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Icontrol_msgs:/opt/ros/noetic/share/control_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg;-Itrajectory_msgs:/opt/ros/noetic/share/trajectory_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(cpr_robot_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/msg/RobotState.msg" NAME_WE)
add_custom_target(_cpr_robot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cpr_robot" "/home/conor/catkin_ws/src/cpr_robot/msg/RobotState.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/msg/ChannelStates.msg" NAME_WE)
add_custom_target(_cpr_robot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cpr_robot" "/home/conor/catkin_ws/src/cpr_robot/msg/ChannelStates.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/srv/GetRobotInfo.srv" NAME_WE)
add_custom_target(_cpr_robot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cpr_robot" "/home/conor/catkin_ws/src/cpr_robot/srv/GetRobotInfo.srv" ""
)

get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/srv/GetJointInfo.srv" NAME_WE)
add_custom_target(_cpr_robot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cpr_robot" "/home/conor/catkin_ws/src/cpr_robot/srv/GetJointInfo.srv" ""
)

get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/srv/RobotCommand.srv" NAME_WE)
add_custom_target(_cpr_robot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cpr_robot" "/home/conor/catkin_ws/src/cpr_robot/srv/RobotCommand.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cpr_robot
)
_generate_msg_cpp(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/msg/ChannelStates.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cpr_robot
)

### Generating Services
_generate_srv_cpp(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/srv/GetRobotInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cpr_robot
)
_generate_srv_cpp(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/srv/GetJointInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cpr_robot
)
_generate_srv_cpp(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/srv/RobotCommand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cpr_robot
)

### Generating Module File
_generate_module_cpp(cpr_robot
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cpr_robot
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(cpr_robot_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(cpr_robot_generate_messages cpr_robot_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/msg/RobotState.msg" NAME_WE)
add_dependencies(cpr_robot_generate_messages_cpp _cpr_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/msg/ChannelStates.msg" NAME_WE)
add_dependencies(cpr_robot_generate_messages_cpp _cpr_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/srv/GetRobotInfo.srv" NAME_WE)
add_dependencies(cpr_robot_generate_messages_cpp _cpr_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/srv/GetJointInfo.srv" NAME_WE)
add_dependencies(cpr_robot_generate_messages_cpp _cpr_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/srv/RobotCommand.srv" NAME_WE)
add_dependencies(cpr_robot_generate_messages_cpp _cpr_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cpr_robot_gencpp)
add_dependencies(cpr_robot_gencpp cpr_robot_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cpr_robot_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cpr_robot
)
_generate_msg_eus(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/msg/ChannelStates.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cpr_robot
)

### Generating Services
_generate_srv_eus(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/srv/GetRobotInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cpr_robot
)
_generate_srv_eus(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/srv/GetJointInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cpr_robot
)
_generate_srv_eus(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/srv/RobotCommand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cpr_robot
)

### Generating Module File
_generate_module_eus(cpr_robot
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cpr_robot
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(cpr_robot_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(cpr_robot_generate_messages cpr_robot_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/msg/RobotState.msg" NAME_WE)
add_dependencies(cpr_robot_generate_messages_eus _cpr_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/msg/ChannelStates.msg" NAME_WE)
add_dependencies(cpr_robot_generate_messages_eus _cpr_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/srv/GetRobotInfo.srv" NAME_WE)
add_dependencies(cpr_robot_generate_messages_eus _cpr_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/srv/GetJointInfo.srv" NAME_WE)
add_dependencies(cpr_robot_generate_messages_eus _cpr_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/srv/RobotCommand.srv" NAME_WE)
add_dependencies(cpr_robot_generate_messages_eus _cpr_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cpr_robot_geneus)
add_dependencies(cpr_robot_geneus cpr_robot_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cpr_robot_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cpr_robot
)
_generate_msg_lisp(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/msg/ChannelStates.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cpr_robot
)

### Generating Services
_generate_srv_lisp(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/srv/GetRobotInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cpr_robot
)
_generate_srv_lisp(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/srv/GetJointInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cpr_robot
)
_generate_srv_lisp(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/srv/RobotCommand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cpr_robot
)

### Generating Module File
_generate_module_lisp(cpr_robot
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cpr_robot
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(cpr_robot_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(cpr_robot_generate_messages cpr_robot_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/msg/RobotState.msg" NAME_WE)
add_dependencies(cpr_robot_generate_messages_lisp _cpr_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/msg/ChannelStates.msg" NAME_WE)
add_dependencies(cpr_robot_generate_messages_lisp _cpr_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/srv/GetRobotInfo.srv" NAME_WE)
add_dependencies(cpr_robot_generate_messages_lisp _cpr_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/srv/GetJointInfo.srv" NAME_WE)
add_dependencies(cpr_robot_generate_messages_lisp _cpr_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/srv/RobotCommand.srv" NAME_WE)
add_dependencies(cpr_robot_generate_messages_lisp _cpr_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cpr_robot_genlisp)
add_dependencies(cpr_robot_genlisp cpr_robot_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cpr_robot_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cpr_robot
)
_generate_msg_nodejs(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/msg/ChannelStates.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cpr_robot
)

### Generating Services
_generate_srv_nodejs(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/srv/GetRobotInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cpr_robot
)
_generate_srv_nodejs(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/srv/GetJointInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cpr_robot
)
_generate_srv_nodejs(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/srv/RobotCommand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cpr_robot
)

### Generating Module File
_generate_module_nodejs(cpr_robot
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cpr_robot
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(cpr_robot_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(cpr_robot_generate_messages cpr_robot_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/msg/RobotState.msg" NAME_WE)
add_dependencies(cpr_robot_generate_messages_nodejs _cpr_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/msg/ChannelStates.msg" NAME_WE)
add_dependencies(cpr_robot_generate_messages_nodejs _cpr_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/srv/GetRobotInfo.srv" NAME_WE)
add_dependencies(cpr_robot_generate_messages_nodejs _cpr_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/srv/GetJointInfo.srv" NAME_WE)
add_dependencies(cpr_robot_generate_messages_nodejs _cpr_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/srv/RobotCommand.srv" NAME_WE)
add_dependencies(cpr_robot_generate_messages_nodejs _cpr_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cpr_robot_gennodejs)
add_dependencies(cpr_robot_gennodejs cpr_robot_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cpr_robot_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cpr_robot
)
_generate_msg_py(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/msg/ChannelStates.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cpr_robot
)

### Generating Services
_generate_srv_py(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/srv/GetRobotInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cpr_robot
)
_generate_srv_py(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/srv/GetJointInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cpr_robot
)
_generate_srv_py(cpr_robot
  "/home/conor/catkin_ws/src/cpr_robot/srv/RobotCommand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cpr_robot
)

### Generating Module File
_generate_module_py(cpr_robot
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cpr_robot
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(cpr_robot_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(cpr_robot_generate_messages cpr_robot_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/msg/RobotState.msg" NAME_WE)
add_dependencies(cpr_robot_generate_messages_py _cpr_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/msg/ChannelStates.msg" NAME_WE)
add_dependencies(cpr_robot_generate_messages_py _cpr_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/srv/GetRobotInfo.srv" NAME_WE)
add_dependencies(cpr_robot_generate_messages_py _cpr_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/srv/GetJointInfo.srv" NAME_WE)
add_dependencies(cpr_robot_generate_messages_py _cpr_robot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/conor/catkin_ws/src/cpr_robot/srv/RobotCommand.srv" NAME_WE)
add_dependencies(cpr_robot_generate_messages_py _cpr_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cpr_robot_genpy)
add_dependencies(cpr_robot_genpy cpr_robot_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cpr_robot_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cpr_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cpr_robot
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(cpr_robot_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(cpr_robot_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET control_msgs_generate_messages_cpp)
  add_dependencies(cpr_robot_generate_messages_cpp control_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cpr_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cpr_robot
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(cpr_robot_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(cpr_robot_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET control_msgs_generate_messages_eus)
  add_dependencies(cpr_robot_generate_messages_eus control_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cpr_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cpr_robot
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(cpr_robot_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(cpr_robot_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET control_msgs_generate_messages_lisp)
  add_dependencies(cpr_robot_generate_messages_lisp control_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cpr_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cpr_robot
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(cpr_robot_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(cpr_robot_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET control_msgs_generate_messages_nodejs)
  add_dependencies(cpr_robot_generate_messages_nodejs control_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cpr_robot)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cpr_robot\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cpr_robot
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(cpr_robot_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(cpr_robot_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET control_msgs_generate_messages_py)
  add_dependencies(cpr_robot_generate_messages_py control_msgs_generate_messages_py)
endif()
