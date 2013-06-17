# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "au_uav_gui: 1 messages, 1 services")

set(MSG_I_FLAGS "-Iau_uav_gui:/home/viki/catkin_ws/src/au_uav_gui/msg;-Istd_msgs:/opt/ros/groovy/share/std_msgs/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

#better way to handle this?
set (ALL_GEN_OUTPUT_FILES_cpp "")

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(au_uav_gui
  /home/viki/catkin_ws/src/au_uav_gui/msg/TelemetryUpdate.msg
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/au_uav_gui
)

### Generating Services
_generate_srv_cpp(au_uav_gui
  /home/viki/catkin_ws/src/au_uav_gui/srv/SendFilePath.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/au_uav_gui
)

### Generating Module File
_generate_module_cpp(au_uav_gui
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/au_uav_gui
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(au_uav_gui_gencpp ALL
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(au_uav_gui
  /home/viki/catkin_ws/src/au_uav_gui/msg/TelemetryUpdate.msg
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/au_uav_gui
)

### Generating Services
_generate_srv_lisp(au_uav_gui
  /home/viki/catkin_ws/src/au_uav_gui/srv/SendFilePath.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/au_uav_gui
)

### Generating Module File
_generate_module_lisp(au_uav_gui
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/au_uav_gui
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(au_uav_gui_genlisp ALL
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(au_uav_gui
  /home/viki/catkin_ws/src/au_uav_gui/msg/TelemetryUpdate.msg
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/au_uav_gui
)

### Generating Services
_generate_srv_py(au_uav_gui
  /home/viki/catkin_ws/src/au_uav_gui/srv/SendFilePath.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/au_uav_gui
)

### Generating Module File
_generate_module_py(au_uav_gui
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/au_uav_gui
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(au_uav_gui_genpy ALL
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)


debug_message(2 "au_uav_gui: Iflags=${MSG_I_FLAGS}")


if(gencpp_INSTALL_DIR)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/au_uav_gui
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(au_uav_gui_gencpp std_msgs_gencpp)

if(genlisp_INSTALL_DIR)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/au_uav_gui
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(au_uav_gui_genlisp std_msgs_genlisp)

if(genpy_INSTALL_DIR)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/au_uav_gui\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/au_uav_gui
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(au_uav_gui_genpy std_msgs_genpy)
