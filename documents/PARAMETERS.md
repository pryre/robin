# Parameters
A parameter listing is dynamically generated on compile. During this time, the parameter headers, initializing functions, update funcitons, and a parameter reference document are generated and placed in `lib/param_generator`.

## Parameter Reference Document
The generated reference document can be [found here](/lib/param_generator/PARAMS.md), or at `lib/param_generator/PARAMS.md`. This document contains a listing of the following headings:
- Name: Parameter name as sent through MAVLINK
- Type: The type of parameter this is (uint, int, float)
- Description: A brief description of what this parameter does in the code
- Default: The default value or option for this parameter
- Unit: Information on the parameter unit (if applicable)
- Options: Provides informatiom about the parameter option (list values, min/max, etc.)
- Reboot: Whether the flight controller needs to be rebooted for the parameter to take effect

## Saving Parameters to the On-Board Flash Memory
To save any parameters that have been changed to make them persist across power downs, they must be saved to the onboard memory. This can be done using the `MAV_CMD_PREFLIGHT_STORAGE` message. In MAVROS:
```sh
rosrun mavros mavcmd long 245 1 0 0 0 0 0 0
```
