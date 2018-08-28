# Simulation
[Back to index](README.md).

## Hardware in the Loop (HitL)
Hardware in the Loop simulation provides methods to use the real hardware with simulation interfaces to test and verify the controller and other methods as if the flight controller on a real platform.

It should be noted that IMU functions are disabled when HitL mode is turned on, meaning that caution should be taken if using your flight controller for both HitL and real-world testing.

#### Device Setup
To enable Hardware in the Loop, it the primary parameter that needs to be changed is `CBRK_HIL`, however it is recommended to set the following additional variables as well:

| **Parameter**   | **Value** | **Reason**                                                     |
| --------------- | ---------:| -------------------------------------------------------------- |
| `CBRK_HIL`      | 1         | Enables HitL funcitons                                         |
| `CBRK_SAFETY`   | 0         | Disable the safety button check to allow easier arming         |
| `SYS_AUTOSTART` | 4001      | Set the mixer to Quad X4, as this is the default simulated UAV |
| `BAUD_RATE_0`   | 921600    | Allows for high data rate transfer of required for HitL        |
| `STRM0_SRV_OUT` | 100       | Sets a high data rate for PWM feedback to simulator            |

#### Simulator Setup
Currently, the simulation interface is provided by the ROS package `robin_gazebo`. To download, compile, and run the simulator, please refer to the [official documentaiton](https://github.com/qutas/robin_gazebo).

#### Running the Simulation
To actually run the simulation, the following steps must be taken:
1. Set the parameters as previously discussed on the flight controller, then reboot it.
2. Configure and run `mavros` such that it connects to the flight controller correctly.
3. Follow the instructions on running the [Robin Gazebo](https://github.com/qutas/robin_gazebo) simulator.
