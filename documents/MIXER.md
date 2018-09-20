# Mixers
[Back to index](README.md).

Mixer profiles currently mirror those available in the [PX4 Airframe Reference](https://dev.px4.io/en/airframes/airframe_reference.html) (as of April 2018). As the PX4 documentation may change in the future, take care and test motor directions before flight!

The mixer profiles can be found in `./include/mixers/`. Custom configurations require modification to both `./include/mixer.h` and `./src/mixer.c`.

## Supported Mixers
Currently, the following mixers are supported:

| **Mixer Decription**             | **PX4 Name**                 | `SYS_AUTOSTART` |
| --------------------             | ------------                 | ---------------:|
| Disabled                         | -                            | 0               |
| Free                             | [Does not exist]             | 999             |
| Fixed-Wing Standard Pass-Through | Standard Plane               | 2100            |
| Quadrotor X4                     | Generic Quadrotor x          | 4001            |
| Quadrotor +4                     | Generic Quadrotor +          | 5001            |
| Hexarotor X6                     | Generic Hexarotor x geometry | 6001            |

As a special note about the "Free" mixer. This will cause the flight controller to not use any of the standard Group 0 mixers, allowing full access to all of the servo/digital outputs through the actuator control interface. This can be especially useful if you wish to use the device as an IO controller without any of the standard controller aspects.

## Running a motor test

1. Ensure that the motors are plugged into the correct ports and props are not connected
2. Ensure correct mixer is selected
3. Disengage the safety switch, but do not arm
4. Send the motor test command (`MAV_CMD_DO_MOTOR_TEST`):
   - GUI: Use one of the tools available [here](TOOLS.md)
   - CLI: If using mavros: `rosrun mavros mavcmd long 209 255 0 0.4 1.5 0 0 0`
