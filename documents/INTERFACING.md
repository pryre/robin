# Interfacing
[Back to index](README.md).

The Robin flight software can be interfaced using MAVLINK, and is compatible to much of the specification. Use of the MAVROS software is highly recommended. A listing of supported MAVLINK telemetry can be [found here](/documents/autogen/MAVLINK_SUPPORT.md)

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

## Actuator Control Input

**If using v0.99.1, please see [this version](https://github.com/qutas/robin/blob/v0.99.1/documents/INTERFACING.md) of the documentation.**


**If using v0.99.2 to v0.99.5, please see [this version](https://github.com/qutas/robin/blob/v0.99.5/documents/INTERFACING.md) of the documentation.**

**If using a version after v0.99.5, please see documentation below.**

The actuator control inputs provides a stable input for controlling PWM driven actuators. The interface exposes 2 input groupings:
1. Group 0: Direct motor mixer control (overrides specific mixer outputs)
2. Group 1: Direct motor mixer additions (adds in additional throttle commands into Group 0)
3. Group 2: RC auxiliary PWM
4. Group 3: RC auxiliary digital
5. Group 4: Offboard auxiliary PWM
6. Group 5: Offboard auxiliary digital

Actuator controls for Group 0 and Group 1 both require streams to be established before control will be passed over. Additionally, nethier stream will be counted as a vaild control input, and as such, needs a Attitude/Rates Control stream established for something to fall back on in case this stream drops out, or a failsafe input is required. The overrides will be re-established once the stream is regained. Lastly, the available mappings for both streams are aligned with the mixer output mappings (i.e. if the mixer `QUAD x` is set, only channels 1 through 4 will be available for overrides).

Group mappings are defined as a bitwise to specify which outputs should be overriden by what groups. The parameters `ACTUATOR_xxPMAP` and `ACTUATOR_xxDMAP` describe the mixing of PWM and digital groups respectively for the RC and offboard inputs. The bitwise mapping is calculated using the LSB as the loweset channel, such that setting bit 0 to `true` will enable channel 1, setting bit 1 to `true` will enable channel 2, etc. As an example, the bitwise mapping works as follows:
- To allow PWM RC actuator control of channel 5:
  - `ACTUATOR_RCPMAP` is set to `16`, which is `0b00010000`
- To allow digital RC actuator control of channel 6 and 7:
  - `ACTUATOR_RCDMAP` is set to `96`, which is `0b01100000`
- To allow digital OB actuator control of channel 8:
  - `ACTUATOR_OBDMAP` is set to `192`, which is `0b10000000`

All enabled outputs will be used when enabled, and by default, will output a "disarmed" value. Digital outputs will be cause the pin to be set `high` for any command greater than 0.

Actuator control for the RC auxiliary groups will begin produce non-disarm values when a valid RC input is received. Actuator control for the offboard auxiliary groups will begin produce non-disarm values when a valid input message is received from a remote platform. Additional settings enabled using the following parameters:
- `ACTUATOR_xx_ARM`: Setting to true (1) will force the actuator outputs to respect arm/disarm conditions (i.e. if disabled, actuators will be able to be controlled without arming the flight controller)
- `ACTUATOR_xx_PDV`: "PWM Disarm Value"; Values in the range of -1 to 1 will be output if `ACTUATOR_xx_ARM` is set true and the flight controller is disarmed, or no valid input has been received.
- `ACTUATOR_xx_DDV`: "Digital Disarm Value"; Values either 0 or 1 (corresponding to `low` and `high`) will be output if `ACTUATOR_xx_ARM` is set true and the flight controller is disarmed, or no valid input has been received.

Additionally, the `ACTUATOR_AUX_ZO` parameter ("Auxiliary Zero Output") can be used to override disarm behaviour of all auxiliary outputs, such that if the flight controller is disarmed (and the groupings respect arm/disarm), zero output will be given, instead of sending "0.0"/"1500 (pwm)". This may be useful if you wish for to respect disarm output, but do not want the actuators to be sent "0.0" on disarm.

The actuator groups overlay one another depending on which outputs are enabled. For example, if the mixer is set to "Quadrotor X4" and RC digital actuator control is set to channel 5 (`16`/`0b00010000`), and the offboard PWM actuator control is set to channels 7 and 8 (`192`/`0b11000000`), the group overlays will look like the following:

| **Group** | `group_mix` | **Actuator 1** | **Actuator 2** | **Actuator 3** | **Actuator 4** | **Actuator 5** | **Actuator 6** | **Actuator 7** | **Actuator 8** |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Motor Mixer | 0 | Motor 1 | Motor 2 | Motor 3 | Motor 4 | Unused | Unused | Unused | Unused |
| Motor Additions | 1 | Blocked | Blocked | Blocked | Blocked | Unused | Unused | Unused | Unused |
| RC PWM | 2 | Blocked | Blocked | Blocked | Blocked | Unused | Unused | Unused | Unused |
| RC Digital | 3 | Blocked | Blocked | Blocked | Blocked | RC [Ch.5] | Unused | Unused | Unused |
| Offboard PWM | 4 | Blocked | Blocked | Blocked | Blocked | Blocked | Unused | Offboard [Ch.7] | Offboard [Ch.8] |
| Offboard Digital | 5 | Blocked | Blocked | Blocked | Blocked | Blocked | Unused | Blocked | Blocked |
| **Final Overlay** | --- | Motor 1 | Motor 2 | Motor 3 | Motor 4 | RC Digital [Ch.5] | --- | Offboard PWM [Ch.7] | Offboard PWM [Ch.8] |

With this setup, actuator 5 is controlled with RC channel 5, and actuators 7 and 8 controlled using the `SET_ACTUATOR_CONTROL_TARGET` message in Mavlink (or using the `actuator_control` plugin in MAVROS). Ensure that `group_mix` variable is set to the desired group to ensure that the actuator control input is correctly registered.
