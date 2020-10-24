#!/bin/sh

if [ $# -eq 0 ]
then
	echo "Usage: sitl_gazebo_run.sh ROBIN_SRC_DIR"
	exit 1
fi

SRC_DIR="$1"

. /usr/share/gazebo/setup.sh

# Options for SITL using just gazebo
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:${SRC_DIR}/build
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:${SRC_DIR}/resources/gazebo
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${SRC_DIR}/resources/gazebo/models
# Disable online model lookup since this is quite experimental and unstable
export GAZEBO_MODEL_DATABASE_URI=

echo ""
echo "Starting gazebo SITL"
echo "+----------------------+"
echo "| Press CTRL+C to exit |"
echo "+----------------------+"
echo ""

gzserver --verbose worlds/robin_quad.world