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

ROBIN_SITL=./robin_posix_udp.elf

# Initialise trap to call cleanup() when CTRL+C (SIGINT) is received
trap "cleanup" 2
echo ""
echo "+----------------------+"
echo "| Press CTRL+C to exit |"
echo "+----------------------+"
echo ""

cd ./build

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
