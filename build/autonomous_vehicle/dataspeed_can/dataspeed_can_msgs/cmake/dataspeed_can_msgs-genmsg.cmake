# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dataspeed_can_msgs: 2 messages, 0 services")

set(MSG_I_FLAGS "-Idataspeed_can_msgs:/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dataspeed_can_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessageStamped.msg" NAME_WE)
add_custom_target(_dataspeed_can_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dataspeed_can_msgs" "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessageStamped.msg" "dataspeed_can_msgs/CanMessage:std_msgs/Header"
)

get_filename_component(_filename "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessage.msg" NAME_WE)
add_custom_target(_dataspeed_can_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dataspeed_can_msgs" "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessage.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(dataspeed_can_msgs
  "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessageStamped.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessage.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dataspeed_can_msgs
)
_generate_msg_cpp(dataspeed_can_msgs
  "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dataspeed_can_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(dataspeed_can_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dataspeed_can_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dataspeed_can_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dataspeed_can_msgs_generate_messages dataspeed_can_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessageStamped.msg" NAME_WE)
add_dependencies(dataspeed_can_msgs_generate_messages_cpp _dataspeed_can_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessage.msg" NAME_WE)
add_dependencies(dataspeed_can_msgs_generate_messages_cpp _dataspeed_can_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dataspeed_can_msgs_gencpp)
add_dependencies(dataspeed_can_msgs_gencpp dataspeed_can_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dataspeed_can_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(dataspeed_can_msgs
  "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessageStamped.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessage.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dataspeed_can_msgs
)
_generate_msg_eus(dataspeed_can_msgs
  "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dataspeed_can_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(dataspeed_can_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dataspeed_can_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(dataspeed_can_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(dataspeed_can_msgs_generate_messages dataspeed_can_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessageStamped.msg" NAME_WE)
add_dependencies(dataspeed_can_msgs_generate_messages_eus _dataspeed_can_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessage.msg" NAME_WE)
add_dependencies(dataspeed_can_msgs_generate_messages_eus _dataspeed_can_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dataspeed_can_msgs_geneus)
add_dependencies(dataspeed_can_msgs_geneus dataspeed_can_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dataspeed_can_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(dataspeed_can_msgs
  "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessageStamped.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessage.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dataspeed_can_msgs
)
_generate_msg_lisp(dataspeed_can_msgs
  "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dataspeed_can_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(dataspeed_can_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dataspeed_can_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dataspeed_can_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dataspeed_can_msgs_generate_messages dataspeed_can_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessageStamped.msg" NAME_WE)
add_dependencies(dataspeed_can_msgs_generate_messages_lisp _dataspeed_can_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessage.msg" NAME_WE)
add_dependencies(dataspeed_can_msgs_generate_messages_lisp _dataspeed_can_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dataspeed_can_msgs_genlisp)
add_dependencies(dataspeed_can_msgs_genlisp dataspeed_can_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dataspeed_can_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(dataspeed_can_msgs
  "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessageStamped.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessage.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dataspeed_can_msgs
)
_generate_msg_nodejs(dataspeed_can_msgs
  "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dataspeed_can_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(dataspeed_can_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dataspeed_can_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(dataspeed_can_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(dataspeed_can_msgs_generate_messages dataspeed_can_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessageStamped.msg" NAME_WE)
add_dependencies(dataspeed_can_msgs_generate_messages_nodejs _dataspeed_can_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessage.msg" NAME_WE)
add_dependencies(dataspeed_can_msgs_generate_messages_nodejs _dataspeed_can_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dataspeed_can_msgs_gennodejs)
add_dependencies(dataspeed_can_msgs_gennodejs dataspeed_can_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dataspeed_can_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(dataspeed_can_msgs
  "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessageStamped.msg"
  "${MSG_I_FLAGS}"
  "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessage.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dataspeed_can_msgs
)
_generate_msg_py(dataspeed_can_msgs
  "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dataspeed_can_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(dataspeed_can_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dataspeed_can_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dataspeed_can_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dataspeed_can_msgs_generate_messages dataspeed_can_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessageStamped.msg" NAME_WE)
add_dependencies(dataspeed_can_msgs_generate_messages_py _dataspeed_can_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg/CanMessage.msg" NAME_WE)
add_dependencies(dataspeed_can_msgs_generate_messages_py _dataspeed_can_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dataspeed_can_msgs_genpy)
add_dependencies(dataspeed_can_msgs_genpy dataspeed_can_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dataspeed_can_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dataspeed_can_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dataspeed_can_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(dataspeed_can_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dataspeed_can_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dataspeed_can_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(dataspeed_can_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dataspeed_can_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dataspeed_can_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(dataspeed_can_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dataspeed_can_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dataspeed_can_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(dataspeed_can_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dataspeed_can_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dataspeed_can_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dataspeed_can_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(dataspeed_can_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
