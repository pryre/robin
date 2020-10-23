#!/bin/sh

source /usr/share/gazebo/setup.sh

# Options for SITL using just gazebo
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:${SRC_DIR}/build
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:${SRC_DIR}/resources/gazebo
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${SRC_DIR}/resources/gazebo/models
# Disable online model lookup since this is quite experimental and unstable
export GAZEBO_MODEL_DATABASE_URI=

gazebo --verbose