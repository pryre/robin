#!/bin/sh

function cleanup() {
    # perform cleanup here
    echo "Shutting down SITL"

    sudo kill $(pgrep -f "robin_sitl") $(pgrep -f "robin_socat")

    exit 0
}

cd ./build

sudo bash -c "exec -a 'robin_socat' socat PTY,link=/dev/ttyROBIN0,raw,echo=0 PTY,link=/dev/ttyUSB0,raw,echo=0" &
PID_SOCAT=$!
echo "Started socat ($PID_SOCAT)"
sleep 0.5
sudo chmod a+rw /dev/ttyROBIN0
sudo chmod a+rw /dev/ttyUSB0

chmod +x ../$1
bash -c "exec -a 'robin_sitl' ../$1" &
PID_ROBIN=$!
echo "Started robin ($PID_ROBIN)"

# Initialise trap to call cleanup() when CTRL+C (SIGINT) is received
trap "cleanup" 2

read -p "Press any key to exit: " -n1 -s

cleanup
