cmake_minimum_required(VERSION 2.8.3)
project(robin)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robin
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(CMD_GIT_VERSION_FLIGHT cd ${PROJECT_SOURCE_DIR} && git describe --abbrev=8 --always | sed 's/^ *//g' | head -c 8)
set(CMD_GIT_VERSION_OS cd ${PROJECT_SOURCE_DIR} && git submodule status | grep BreezySTM32 | sed 's/^ *//g' | head -c 8)
set(CMD_GIT_VERSION_MAVLINK cd ${PROJECT_SOURCE_DIR} && git submodule status | grep mavlink | sed 's/^ *//g' | head -c 8)
set(CMD_EEPROM_CONF_VERSION date +'%s')

execute_process(COMMAND ${CMD_GIT_VERSION_FLIGHT} RESULT_VARIABLE GIT_VERSION_FLIGHT)
execute_process(COMMAND ${CMD_GIT_VERSION_OS} RESULT_VARIABLE GIT_VERSION_OS)
execute_process(COMMAND ${CMD_GIT_VERSION_MAVLINK} RESULT_VARIABLE GIT_VERSION_MAVLINK)
execute_process(COMMAND ${CMD_EEPROM_CONF_VERSION} RESULT_VARIABLE EEPROM_CONF_VERSION)

file(GLOB PARAM_DEFINITIONS ${PROJECT_SOURCE_DIR}/lib/param_generator/definitions/*.yaml)

add_custom_command(
    OUTPUT  param_gen.h param_gen.c
    COMMAND python3 ${PROJECT_SOURCE_DIR}/lib/scripts/param_generator/gen_params.py ${PROJECT_SOURCE_DIR}/resources/param_definitions/ ${CMAKE_CURRENT_BINARY_DIR}/
    DEPENDS ${PARAM_DEFINITIONS}
)

add_compile_options(
	-DGIT_VERSION_FLIGHT_STR=\"${GIT_VERSION_FLIGHT}\"
	-DGIT_VERSION_OS_STR=\"${GIT_VERSION_OS}\"
	-DGIT_VERSION_MAVLINK_STR=\"${GIT_VERSION_MAVLINK}\"
	-DEEPROM_CONF_VERSION_STR=\"${EEPROM_CONF_VERSION}\"
	-DFIXMATH_NO_CACHE
	#-Wall -pedantic -Wextra -Wshadow -Wunsafe-loop-optimizations
	#-Wno-char-subscripts -Wno-sign-compare -Wno-unused-parameter -Wno-unused-result
	#-ffunction-sections
	#-fdata-sections
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  lib
  lib/libfixmath/libfixmath
  lib/libfixmatrix
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/robin.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set(ROBIN_SRC_MAINS
	src/drivers/posix_ros/robin_node.cpp
	src/calibration.c
	src/controller.c
	src/estimator.c
	src/mavlink_receive.c
	src/mavlink_system.c
	src/mavlink_transmit.c
	src/mixer.c
	src/params.c
	src/pid_controller.c
	src/profiler.c
	src/robin_itoa.c
	src/run.c
	src/safety.c
	src/sensors.c
)

set(ROBIN_SRC_CALS
	src/calibrations/calibrations_accel.c
	src/calibrations/calibrations_barometer.c
	src/calibrations/calibrations_esc.c
	src/calibrations/calibrations_ground_pressure.c
	src/calibrations/calibrations_gyro.c
	src/calibrations/calibrations_interference.c
	src/calibrations/calibrations_level_horizon.c
	src/calibrations/calibrations_magnetometer.c
	src/calibrations/calibrations_rc.c
)

set(ROBIN_SRC_DRIVERS
	src/drivers/drv_comms.c
	src/drivers/drv_status_io.c
	src/drivers/posix_ros/drv_system.cpp
	src/drivers/posix_udp/drv_comms.c
	src/drivers/posix_common/drv_cmd_args.c
	src/drivers/posix_common/drv_flash.c
	src/drivers/posix_common/drv_pwm_ppm.c
	src/drivers/posix_common/drv_sensors.c
	src/drivers/posix_common/drv_status_io.c
)

set(ROBIN_SRC_MAVLINK_HANDLERS
	src/mavlink_handlers/mavlink_handle_att_pos_mocap.c
	src/mavlink_handlers/mavlink_handle_command_long.c
	src/mavlink_handlers/mavlink_handle_command_long_component_arm_disarm.c
	src/mavlink_handlers/mavlink_handle_command_long_do_motor_test.c
	src/mavlink_handlers/mavlink_handle_command_long_get_home_position.c
	src/mavlink_handlers/mavlink_handle_command_long_preflight_calibration.c
	src/mavlink_handlers/mavlink_handle_command_long_preflight_storage.c
	src/mavlink_handlers/mavlink_handle_command_long_reboot_shutdown.c
	src/mavlink_handlers/mavlink_handle_command_long_request_autopilot_capabilities.c
	src/mavlink_handlers/mavlink_handle_command_long_request_protocol_version.c
	src/mavlink_handlers/mavlink_handle_command_long_set_mode.c
	src/mavlink_handlers/mavlink_handle_heartbeat.c
	src/mavlink_handlers/mavlink_handle_hil_sensor.c
	src/mavlink_handlers/mavlink_handle_mission_request_list.c
	src/mavlink_handlers/mavlink_handle_param_request_list.c
	src/mavlink_handlers/mavlink_handle_param_request_read.c
	src/mavlink_handlers/mavlink_handle_param_set.c
	src/mavlink_handlers/mavlink_handle_ping.c
	src/mavlink_handlers/mavlink_handle_set_actuator_control_target.c
	src/mavlink_handlers/mavlink_handle_set_attitude_target.c
	src/mavlink_handlers/mavlink_handle_set_mode.c
	src/mavlink_handlers/mavlink_handle_timesync.c
)

set(ROBIN_SRC_LIBFIXMATH
	lib/libfixmath/libfixmath/fix16.c
	lib/libfixmath/libfixmath/fix16_exp.c
	lib/libfixmath/libfixmath/fix16_sqrt.c
	lib/libfixmath/libfixmath/fix16_str.c
	lib/libfixmath/libfixmath/fix16_trig.c
	lib/libfixmath/libfixmath/fract32.c
	lib/libfixmath/libfixmath/uint32.c
)

set(ROBIN_SRC_LIBFIXMATRIX
	lib/libfixmatrix/fixarray.c
	lib/libfixmatrix/fixmatrix.c
	lib/libfixmatrix/fixquat.c
	lib/libfixmatrix/fixstring.c
	lib/libfixmatrix/fixvector2d.c
	lib/libfixmatrix/fixvector3d.c
)

set(ROBIN_SRC_PARAMGEN
	param_gen.c
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node
	${ROBIN_SRC_MAINS}
	${ROBIN_SRC_CALS}
	${ROBIN_SRC_DRIVERS}
	${ROBIN_SRC_MAVLINK_HANDLERS}
	${ROBIN_SRC_LIBFIXMATH}
	${ROBIN_SRC_LIBFIXMATRIX}
	${ROBIN_SRC_PARAMGEN}
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robin.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
