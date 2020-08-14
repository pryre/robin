# Tuning
[Back to index](README.md).

Some of the followings note have been adapted from the [PX4 Tuning Guide](https://docs.px4.io/v1.10/en/config_mc/pid_tuning_guide_multicopter.html) as a reference. Please note that PX4 gains are not directly usable with _robin_.


## Pre-Tuning Notes
- All gains should be increased very slowly as large gains may cause dangerous oscillations! Typically increase gains by 20-30% per iteration, reducing to 5-10% for final fine tuning.
- Land before changing a parameter. Slowly increase the throttle and check for oscillations.
- Tune the vehicle around the hovering thrust point, and ensure you have correctly disabled `PWM_NONLINEAR_M` if your ESC(s) have a linear thrust profile --- Note: Most low-cost ESCs do not have a linear thrust profile and require `PWM_NONLINEAR_M` to be left enabled.

## Understanding the Gains

#### P Gain
The P (proportional) gain is used to minimize the tracking error. It is responsible for a quick response and acts like a spring that "pulls" the actual state closer to the desired state.

For example, in Acro mode, the desired state at centre stick is 0 rad/s of angular velocity, so if the system is spinning at 5 rad/s, then the P gain will "pull against" this motion to slow the system down (in this case the control signal would be: u = P*(0 - 5))

Thus, the P gain should be set as high as possible, but without introducing oscillations, as this will give the "quickest" response.
- If the P gain is too high: you will see high-frequency oscillations.
- If the P gain is too low:
  - the vehicle will react slowly to input changes.
  - In Acro mode the vehicle will drift, and you will constantly need to correct to keep it level.

#### D Gain
The D (derivative) gain is used for damping. It is used to internally oppose any motion in the system. In the spring example, this will act as a literal dampener. In a practical example, the D gain will adjust the control signal to try and "slow the system down" depending on how fast the state is changing.

For example, in Acro mode, the system may be close to stationary and have some minor changes in angular velocity. The state that we are tracking in this case _is_ angular velocity, so the derivative gain tries to slow down how much this changes (_i.e._ it will slow down the acceleration of the system). If the angular velocity is zero (P gain), and the anglar acceleration is zero (D gain), then the system will hold exactly where it is!

Thus, the D gain is useful to reduce high-frequency oscillations and increase the performance when the system is tuned to a high P gain. As it acts on the derivative, you could also think of it as "friction" to the system gaining angular velocity (but this analogy doesn't hold to well when trying to control velocty, so maybe ignore that!).

The D gain is usually require for aircraft control, but should only be set as high as needed to avoid overshoots.
- If the D gain is very much too high: the system will become uncontrollable because the D term amplifies noise.
- If the D gain is too high: the motors become twitchy (and maybe hot), because the D term amplifies noise.
- If the D gain is too low: you see overshoots after a step-input, but system should still respond well and follow stick input in general.

#### I Gain
The I (integral) gain keeps a memory of the error. The I term increases when the desired rate is not reached over some time. It is important (especially when flying Acro mode), but it should not be set too high.

---

**Note:** most small-to-mid sized systems (or those where C.o.G. is close to the body centre in all axes) will operate to a reasonable degree with no integral gain set. Integral gain should be set to zero for the initial tuning procedure then slowly increased as/if required.

---

While tuning the I gain:
- If the I gain is too high: you will see slow oscillations (<5Hz).
- If the I gain is too low: the system will

## Tuning the Rate Controllers

#### Finding the Right Range for the System
The specific combination of ESCs, motors, servos, and/or control surfaces of a system will give significantly different responses. However, once the correct range of values is found, a system can be initially tuned to a relatively similar ratio of gains to other systems.

For a multirotor, the following gain ratios are suggested as a starting point for entering your gains (refer to the "Fine Tuning" section below before setting these values!):
- Roll/Pitch:
  - `P gain: 1.00`
  - `I gain: 0.20`
  - `D gain: 0.01`
- Yaw:
  - `P gain: 1.00`
  - `I gain: 0.30`
  - `D gain: 0.00` (typically not needed)

An example tuning for a generic multirotor (after fine tuning) might be:
- Roll/Pitch:
  - `P gain: 5.00`
  - `I gain: 0.80`
  - `D gain: 0.01`
- Yaw:
  - `P gain: 5.00`
  - `I gain: 1.00`
  - `D gain: 0.00`

#### Fine Tuning
It is recommended to adjust the values in the following order:
1. Set I and D gains to 0.
2. Tune P gain to give a response that tracks the commanded state.
3. Increase P gain in 10% increments until slight oscillations appear.
4. Set D gain at approximate ratio described above.
5. If oscillation disapear, continue slowly increasing P gain.
6. If oscllations reappear, increase D gain by 10% until oscillations disapear (usually only 1-2 steps).
7. If motors sound overly twitchy or are getting noticably warm between resets/tests, reduce gains back to last reasonable values and leave there. **This should point should give you a good tuning.**
8. Begin to adjust the I gain:
  1. Set I gain at approximate ratio described above.
  2. Tuning is best tested in Acro mode, by tilting the vehicle to one side about 45 degrees, and attempting to keep this attitude.
  3. It should keep the same angle. If it drifts back, increase the I gain by 10%.
  4. A low I gain is also visible by viewing the reported telemetry (refer to the Mavlink messages `ATTITUDE_TARGET` for reference and `HIGHRES_IMU` for state). A low I gain would show an offset between the reference and the actual body rates over a longer time (1-5 seconds).

## Tuning the Attitude Controller
The attitude controller follows a much simpler process as it is only a proportional controller. The theory being that if the rate response is good, then commanding "hold-still at this attitude" will give the intended response, thus we only need to "guide" the attitude (with the P gain) to the correct position, then hold there.

The result is that the attitude controller _typically does not require tuning_. If the attitude controller is aggressively tuned, or the system oscillates around a specific attitude with a a period of 1-2Hz (but is stable), then you may consider tuning down the P gain for the attitude controller.

To tune, first ensure that the system response appropriately in rate/Acro mode, then adjust the P gain of the attitude controller in stabilised mode until the response is oscillation-free and appropriately responsive to stick inputs.

#### Relevant Parameters
- `MC_ANGLE_P`: the P gain for the attitude controller. Values of `3.5` to `4.5` are suitable for most systems.
- `MC_YAW_W`: adjusts the amount of "yaw influence" the system will utilise during attitude correction. Values of `0.6` to `0.8` will favour faster roll-pitch manoeuvres over slower yaw manoeuvres to reduce attitude error. A value of `1.0` will still allow the system to operate fine.
- `MAX_ROLL_A` and `MAX_PITCH_A`: Limits the maximum commanded roll and pitch angles _from the RC transmitter during stabilised mode_.

## Resources for Better Understanding the Tuning Process
- [Wikipedia page on PID Controllers (and the effects of each gain)](https://en.wikipedia.org/wiki/PID_controller)
- [PX4 tuning guide (on which the controllers in _robin_ ware based)](https://docs.px4.io/v1.10/en/config_mc/pid_tuning_guide_multicopter.html] --- Note: the tuning parameters are different between the two systems, so values are not directly applicable. Only the actual tuning process is relevant.
- [A video guide that follows a similar process to above](https://www.youtube.com/watch?v=aq1jXHMiJgg) --- Note: the tuning parameters are different between the two systems, so values are not directly applicable. Only the actual tuning process is relevant.
