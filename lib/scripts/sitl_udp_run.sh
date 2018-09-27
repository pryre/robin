#!/bin/sh

cleanup() {
    # perform cleanup here
    echo "Shutting down SITL"

    sudo kill $(pgrep -f "robin_sitl")

    exit 0
}

sudo -v

ROBIN_SITL=./robin_posix_udp.elf


# Initialise trap to call cleanup() when CTRL+C (SIGINT) is received
trap "cleanup" 2
echo "+----------------------+"
echo "| Press CTRL+C to exit |"
echo "+----------------------+"

cd ./build

chmod +x $ROBIN_SITL
bash -c "exec -a 'robin_sitl' $ROBIN_SITL" &
PID_ROBIN=$!
echo "Started robin ($PID_ROBIN)"

sleep infinity

cleanup
