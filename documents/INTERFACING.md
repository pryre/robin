# Interfacing
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
