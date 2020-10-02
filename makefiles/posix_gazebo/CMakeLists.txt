cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(Robin)

set(PROJECT_TOP_DIR ../..)

set(CMD_GIT_VERSION_FLIGHT cd ${PROJECT_TOP_DIR} && git describe --abbrev=8 --always | sed 's/^ *//g' | head -c 8)
set(CMD_GIT_VERSION_OS cd ${PROJECT_TOP_DIR} && git submodule status | grep BreezySTM32 | sed 's/^ *//g' | head -c 8)
set(CMD_GIT_VERSION_MAVLINK cd ${PROJECT_TOP_DIR} && git submodule status | grep mavlink | sed 's/^ *//g' | head -c 8)
set(CMD_EEPROM_CONF_VERSION date +'%s')

execute_process(COMMAND ${CMD_GIT_VERSION_FLIGHT} RESULT_VARIABLE GIT_VERSION_FLIGHT)
execute_process(COMMAND ${CMD_GIT_VERSION_OS} RESULT_VARIABLE GIT_VERSION_OS)
execute_process(COMMAND ${CMD_GIT_VERSION_MAVLINK} RESULT_VARIABLE GIT_VERSION_MAVLINK)
execute_process(COMMAND ${CMD_EEPROM_CONF_VERSION} RESULT_VARIABLE EEPROM_CONF_VERSION)

file(GLOB PARAM_DEFINITIONS ${PROJECT_TOP_DIR}/resources/param_definitions/*.yaml)

add_custom_command(
    OUTPUT  param_gen.h param_gen.c
    COMMAND python3 ${PROJECT_TOP_DIR}/lib/scripts/param_generator/gen_params.py ${PROJECT_TOP_DIR}/resources/param_definitions/ ${CMAKE_CURRENT_BINARY_DIR}/
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

include_directories(
  include
  ${PROJECT_TOP_DIR}/build
  ${PROJECT_TOP_DIR}/include
  ${PROJECT_TOP_DIR}/lib
  ${PROJECT_TOP_DIR}/lib/libfixmath/libfixmath
  ${PROJECT_TOP_DIR}/lib/libfixmatrix
)

# Get all of our source files except the gazebo plugin file
FILE(GLOB ROBIN_SRC_MAINS
	${PROJECT_TOP_DIR}/src/*.c
)

FILE(GLOB ROBIN_SRC_CALS ${PROJECT_TOP_DIR}/src/calibrations/**.c)
FILE(GLOB ROBIN_SRC_CONTROLLERS ${PROJECT_TOP_DIR}/src/controllers/**.c)
FILE(GLOB ROBIN_SRC_DRIVERS ${PROJECT_TOP_DIR}/src/drivers/*.c)
FILE(GLOB ROBIN_SRC_MAVLINK_HANDLERS ${PROJECT_TOP_DIR}/src/mavlink_handlers/**.c)

set(ROBIN_SRC_DRIVERS
	${PROJECT_TOP_DIR}/src/drivers/drv_comms.c
	${PROJECT_TOP_DIR}/src/drivers/drv_status_io.c
	${PROJECT_TOP_DIR}/src/drivers/posix_gazebo/drv_comms.c
	${PROJECT_TOP_DIR}/src/drivers/posix_gazebo/drv_pwm_ppm.c
	${PROJECT_TOP_DIR}/src/drivers/posix_gazebo/drv_status_io.c
	${PROJECT_TOP_DIR}/src/drivers/posix_gazebo/drv_flash.c 
	${PROJECT_TOP_DIR}/src/drivers/posix_gazebo/drv_sensors.c
	${PROJECT_TOP_DIR}/src/drivers/posix_gazebo/drv_system.cpp
	${PROJECT_TOP_DIR}/src/drivers/posix_gazebo/robin_plugin.cpp
)

set(ROBIN_SRC_LIBFIXMATH
	${PROJECT_TOP_DIR}/lib/libfixmath/libfixmath/fix16.c
	${PROJECT_TOP_DIR}/lib/libfixmath/libfixmath/fix16_exp.c
	${PROJECT_TOP_DIR}/lib/libfixmath/libfixmath/fix16_sqrt.c
	${PROJECT_TOP_DIR}/lib/libfixmath/libfixmath/fix16_str.c
	${PROJECT_TOP_DIR}/lib/libfixmath/libfixmath/fix16_trig.c
	${PROJECT_TOP_DIR}/lib/libfixmath/libfixmath/fract32.c
	${PROJECT_TOP_DIR}/lib/libfixmath/libfixmath/uint32.c
)

set(ROBIN_SRC_LIBFIXMATRIX
	${PROJECT_TOP_DIR}/lib/libfixmatrix/fixarray.c
	${PROJECT_TOP_DIR}/lib/libfixmatrix/fixmatrix.c
	${PROJECT_TOP_DIR}/lib/libfixmatrix/fixquat.c
	${PROJECT_TOP_DIR}/lib/libfixmatrix/fixstring.c
	${PROJECT_TOP_DIR}/lib/libfixmatrix/fixvector2d.c
	${PROJECT_TOP_DIR}/lib/libfixmatrix/fixvector3d.c
)

set(ROBIN_SRC_PARAMGEN
	${PROJECT_TOP_DIR}/build/param_gen.c
)

add_library(${PROJECT_NAME}Plugin
	SHARED
	${ROBIN_SRC_MAINS}
	${ROBIN_SRC_CALS}
	${ROBIN_SRC_CONTROLLERS}
	${ROBIN_SRC_DRIVERS}
	${ROBIN_SRC_MAVLINK_HANDLERS}
	${ROBIN_SRC_LIBFIXMATH}
	${ROBIN_SRC_LIBFIXMATRIX}
	${ROBIN_SRC_PARAMGEN}
)

target_link_libraries(${PROJECT_NAME}Plugin
)

#Handle Gazebo Interface
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(Gazebo${PROJECT_NAME}Plugin
	SHARED
	${PROJECT_TOP_DIR}/src/drivers/posix_gazebo/gazebo_robin_plugin.cpp
)

target_link_libraries(Gazebo${PROJECT_NAME}Plugin
  ${PROJECT_NAME}Plugin
  ${GAZEBO_LIBRARIES}
)

