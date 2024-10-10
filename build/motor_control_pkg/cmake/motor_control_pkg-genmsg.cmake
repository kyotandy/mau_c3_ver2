# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "motor_control_pkg: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(motor_control_pkg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/kyota/mau_c3_ver2/src/motor_control_pkg/srv/ModbusWrite.srv" NAME_WE)
add_custom_target(_motor_control_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motor_control_pkg" "/home/kyota/mau_c3_ver2/src/motor_control_pkg/srv/ModbusWrite.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(motor_control_pkg
  "/home/kyota/mau_c3_ver2/src/motor_control_pkg/srv/ModbusWrite.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_control_pkg
)

### Generating Module File
_generate_module_cpp(motor_control_pkg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_control_pkg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(motor_control_pkg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(motor_control_pkg_generate_messages motor_control_pkg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kyota/mau_c3_ver2/src/motor_control_pkg/srv/ModbusWrite.srv" NAME_WE)
add_dependencies(motor_control_pkg_generate_messages_cpp _motor_control_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motor_control_pkg_gencpp)
add_dependencies(motor_control_pkg_gencpp motor_control_pkg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motor_control_pkg_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(motor_control_pkg
  "/home/kyota/mau_c3_ver2/src/motor_control_pkg/srv/ModbusWrite.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motor_control_pkg
)

### Generating Module File
_generate_module_eus(motor_control_pkg
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motor_control_pkg
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(motor_control_pkg_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(motor_control_pkg_generate_messages motor_control_pkg_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kyota/mau_c3_ver2/src/motor_control_pkg/srv/ModbusWrite.srv" NAME_WE)
add_dependencies(motor_control_pkg_generate_messages_eus _motor_control_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motor_control_pkg_geneus)
add_dependencies(motor_control_pkg_geneus motor_control_pkg_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motor_control_pkg_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(motor_control_pkg
  "/home/kyota/mau_c3_ver2/src/motor_control_pkg/srv/ModbusWrite.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_control_pkg
)

### Generating Module File
_generate_module_lisp(motor_control_pkg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_control_pkg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(motor_control_pkg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(motor_control_pkg_generate_messages motor_control_pkg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kyota/mau_c3_ver2/src/motor_control_pkg/srv/ModbusWrite.srv" NAME_WE)
add_dependencies(motor_control_pkg_generate_messages_lisp _motor_control_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motor_control_pkg_genlisp)
add_dependencies(motor_control_pkg_genlisp motor_control_pkg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motor_control_pkg_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(motor_control_pkg
  "/home/kyota/mau_c3_ver2/src/motor_control_pkg/srv/ModbusWrite.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motor_control_pkg
)

### Generating Module File
_generate_module_nodejs(motor_control_pkg
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motor_control_pkg
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(motor_control_pkg_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(motor_control_pkg_generate_messages motor_control_pkg_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kyota/mau_c3_ver2/src/motor_control_pkg/srv/ModbusWrite.srv" NAME_WE)
add_dependencies(motor_control_pkg_generate_messages_nodejs _motor_control_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motor_control_pkg_gennodejs)
add_dependencies(motor_control_pkg_gennodejs motor_control_pkg_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motor_control_pkg_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(motor_control_pkg
  "/home/kyota/mau_c3_ver2/src/motor_control_pkg/srv/ModbusWrite.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_control_pkg
)

### Generating Module File
_generate_module_py(motor_control_pkg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_control_pkg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(motor_control_pkg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(motor_control_pkg_generate_messages motor_control_pkg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kyota/mau_c3_ver2/src/motor_control_pkg/srv/ModbusWrite.srv" NAME_WE)
add_dependencies(motor_control_pkg_generate_messages_py _motor_control_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motor_control_pkg_genpy)
add_dependencies(motor_control_pkg_genpy motor_control_pkg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motor_control_pkg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_control_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_control_pkg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(motor_control_pkg_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motor_control_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motor_control_pkg
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(motor_control_pkg_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_control_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_control_pkg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(motor_control_pkg_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motor_control_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motor_control_pkg
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(motor_control_pkg_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_control_pkg)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_control_pkg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_control_pkg
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(motor_control_pkg_generate_messages_py std_msgs_generate_messages_py)
endif()
