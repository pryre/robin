# Interfacing
[Back to index](README.md).

The Robin flight software can be interfaced using MAVLINK, and is compatible to much of the specification. Use of the MAVROS software is highly recommended.

## Attitude/Rates Control
The primary input for controlling the UAV is through the `SET_ATTITUDE_TARGET` message. If using MAVROS, it is recommended that you use the plugin `setpoint_raw` for sending commands. All `type_mask` options are supported for this message, so it is possible to do full attitude control, full rates control, or anything in between.

The `TIMEOUT_OB_CTRL` and `STRM_NUM_OB_C` parameters can be used to set the timeout period and the number of messages needed to establish a stream respectively.

In the event of a stream timing out, the flight controller will enter a failsafe state. In this state, it will attempt to hold it's current attitude (0 rates) and apply the `FAILSAFE_THRTL` parameter as the throttle input.

---
**Note:** Ensure that the `FAILSAFE_THRTL` throttle value will cause the UAV to come to some form of landing. If it is set too high, your UAV may fly off!

---

## External Heading Estimation
The primary input for providing an external heading estimation is through the `ATT_POS_MOCAP` message. If using mavros, is is recommended to use the plugin `mocap_pose_estimate` for sending an estimated pose to the UAV, from which the external heading estimation will be extracted.

The `TIMEOUT_EXT_P` and `STRM_NUM_EXT_P` parameters can be used to set the timeout period and the number of messages needed to establish a stream respectively.

The `FSE_EXT_HDG_W` parameter can be used to define the weighting of the heading reading to be used in the correction steps. Setting this closer to 0.0 will cause it to be "trusted less", allowing for some deviations, where as setting it closer to 1.0 will cause it to be "trusted more", causing a quicker snap to the estimated heading.

In the event of a stream timing out, the estimator will simply ignore this functionallity until the stream is re-established.

## PWM Overrides
As a secondary form of control, the `RC_CHANNELS_OVERRIDE` message can be used to directly control the PWM outputs. Arming conditions will be adhered to, and additionally, `MOTOR_PWM_IDLE` will also override the requested PWM (so disable the idle if this will be an issue).

The `TIMEOUT_RC_CTRL` and `STRM_NUM_RC_C` parameters can be used to set the timeout period and the number of messages needed to establish a stream respectively.

This message will only not count towards failsafe protection, and as such, needs a Attitude/Rates Control stream established for something to fall back on if this stream drops out. The overrides will be re-established once the stream is regained.

## Actuator Control Input

**If using v0.99.1, please see [this version](https://github.com/qutas/robin/blob/v0.99.1/documents/INTERFACING.md) of the documentation.**

**If using v0.99.2 and onwards, please see documentation below.**

The actuator control inputs provides a stable input for controlling PWM driven actuators. The interface exposes 2 input groupings:
1. Group 0: Motor mixer input (Currently unimplemented)
2. Group 1: RC auxilary input
3. Group 2: Offboard auxilary input

Direct actuator control for the offboard group `ACTUATOR_RC_*` is enabled when any of the `RC_MAP_AUX*` parameters are set, and act as a partial overlay depending on which auxilaries are set. The following parameters can be used for additional configuration:
- `ACTUATOR_RC_ARM`: Setting to true (1) will force the actuator outputs to respect arm/disarm conditions (i.e. if disabled, actuators will be able to be controlled without arming the flight controller)
- `ACTUATOR_RC_ODV`: "Ouput Disarm Value"; Values in the range of -1 to 1 will be output if `ACTUATOR_RC_ARM` is set true and the flight controller is disarmed.

Direct actuator control for the offboard group `ACTUATOR_OB_*` is enabled when an initial input message is received, with additional settings enabled using the following parameters:
- `ACTUATOR_OB_ARM`: Setting to true (1) will force the actuator outputs to respect arm/disarm conditions (i.e. if disabled, actuators will be able to be controlled without arming the flight controller)
- `ACTUATOR_OB_ODV`: "Ouput Disarm Value"; Values in the range of -1 to 1 will be output if `ACTUATOR_RC_ARM` is set true and the flight controller is disarmed.

Additionally, the `ACTUATOR_AUX_ZO` parameter ("Auxilary Zero Output") can be used to override disarm behaviour of all auxilary outputs, such that if the flight controller is disarmed (and the groupings respect arm/disarm), zero output will be given, instead of sending "0.0"/"1500 (pwm)". This may be useful if you wish for to respect disarm output, but do not want the actuators to be sent "0.0" on disarm.

The actuator groups overlay one another depending on which outputs are enabled. For example, if the mixer is set to "Quadrotor X4" and `RC_MAP_AUX2` is set to RC input channel 5, and an offboard actuator set has been received, the group overlays will look like the following:

| **Group** | `group_mix` | **Actuator 1** | **Actuator 2** | **Actuator 3** | **Actuator 4** | **Actuator 5** | **Actuator 6** | **Actuator 7** | **Actuator 8** |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Motor Mixer | 0 | Motor 1 | Motor 2 | Motor 3 | Motor 4 | Unused | Unused | Unused | Unused |
| RC Aux. | 1 | Blocked | Blocked | Blocked | Blocked | Unused | RC [Ch.5] | Unused | Unused |
| Offboard Aux. | 2 | Blocked | Blocked | Blocked | Blocked | Free | Blocked | Free | Free |
| **Final Overlay** | --- | Motor 1 | Motor 2 | Motor 3 | Motor 4 | Offboard [Ch.5] | RC [Ch.5] |  Offboard [Ch.7] | Offboard [Ch.8] |

With this setup, actuator 6 is controlled with RC channel 5, and actuators 5, 7, and 8 controlled using the `SET_ACTUATOR_CONTROL_TARGET` message in Mavlink (or using the `actuator_control` plugin in MAVROS). Ensure that `group_mix` variable is set to the desired group to ensure that the actuator control input is correctly registered.
