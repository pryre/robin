#!/bin/sh

cleanup() {
    # perform cleanup here
    echo "Shutting down SITL"

	PID_ROBIN_ALL=$(pgrep -f "robin_sitl")
	if [ $PID_ROBIN_ALL ]; then
	    echo "Killing all instances of robin_sitl"
	    kill $PID_ROBIN_ALL
	fi

    exit 0
}

sudo -v

ROBIN_SITL=./robin_posix_serial.elf
SERIAL_PORT_0=$1
SERIAL_PORT_1=$2

# Initialise trap to call cleanup() when CTRL+C (SIGINT) is received
trap "cleanup" 2
echo "+----------------------+"
echo "| Press CTRL+C to exit |"
echo "+----------------------+"

cd ./build

if [ -n "$SERIAL_PORT_0" ]
then
	sudo bash -c "exec -a 'robin_socat_0' socat PTY,link=/dev/ttyROBIN0,raw,echo=0 PTY,link=$SERIAL_PORT_0,raw,echo=0" &
	PID_SOCAT_0=$!
	echo "Started socat[0] ($PID_SOCAT_0)"

	sleep 0.5
	sudo chmod a+rw /dev/ttyROBIN0
	sudo chmod a+rw $SERIAL_PORT_0
fi

if [ -n "$SERIAL_PORT_1" ]
then
	sudo bash -c "exec -a 'robin_socat_1' socat PTY,link=/dev/ttyROBIN1,raw,echo=0 PTY,link=$SERIAL_PORT_1,raw,echo=0" &
	PID_SOCAT_1=$!
	echo "Started socat[1] ($PID_SOCAT_1)"

	sleep 0.5
	sudo chmod a+rw /dev/ttyROBIN1
	sudo chmod a+rw $SERIAL_PORT_1
fi

chmod +x $ROBIN_SITL
bash -c "exec -a 'robin_sitl' $ROBIN_SITL" &
PID_ROBIN=$!
echo "Started robin ($PID_ROBIN)"

while [ 1 ]; do
	#Lock terminal here, but check that robin is still alive
	if ! kill -0 $PID_ROBIN 2>/dev/null; then
		#If not running, then exit and perform cleanup
		break
	fi

	sleep 0.5
done

cleanup
