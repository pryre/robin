# Tuning
[Back to index](README.md).

Some of the followings note have been adapted from the [PX4 Tuning Guide](https://docs.px4.io/v1.10/en/config_mc/pid_tuning_guide_multicopter.html) as a reference. Please note that PX4 gains are not directly usable with __robin__.


## Pre-Tuning Notes
- All gains should be increased very slowly as large gains may cause dangerous oscillations! Typically increase gains by 20-30% per iteration, reducing to 5-10% for final fine tuning.
- Land before changing a parameter. Slowly increase the throttle and check for oscillations.
- Tune the vehicle around the hovering thrust point, and ensure you have correctly disabled `PWM_NONLINEAR_M` if your ESC(s) have a linear thrust profile --- Note: Most low-cost ESCs do not have a linear thrust profile and require `PWM_NONLINEAR_M` to be left enabled.

## Understanding the Gains

#### P Gain
The P (proportional) gain is used to minimize the tracking error. It is responsible for a quick response and thus should be set as high as possible, but without introducing oscillations.

- If the P gain is too high: you will see high-frequency oscillations.
- If the P gain is too low:
  - the vehicle will react slowly to input changes.
  - In Acro mode the vehicle will drift, and you will constantly need to correct to keep it level.

#### D Gain
The D (derivative) gain is used for dampening. It is required but should be set only as high as needed to avoid overshoots.

- If the D gain is too high: the motors become twitchy (and maybe hot), because the D term amplifies noise.
- If the D gain is too low: you see overshoots after a step-input.

#### I Gain
The I (integral) gain keeps a memory of the error. The I term increases when the desired rate is not reached over some time. It is important (especially when flying Acro mode), but it should not be set too high.

- If the I gain is too high: you will see slow oscillations.
- If the I gain is too low: this is best tested in Acro mode, by tilting the vehicle to one side about 45 degrees, and keeping it like that. It should keep the same angle. If it drifts back, increase the I gain. A low I gain is also visible in a log, when there is an offset between the desired and the actual rate over a longer time.

---

**Note:** most smaller systems will operate to a reasonable degree with no integral gain set. Integral gain should be set to zero for the initial tuning procedure then slowly increased as/if required.

---

Typical values are between 0.3 and 0.5, and the pitch gain usually needs to be a bit higher.
## Rate Controllers

## Attitude Controller
The attitude controller follows a much simpler process as it is only a proportional controller. The theory being that if the rate response is good, then commanding "hold-still at this attitude" will give the intended response, thus we only need to "guide" the attitude (with the P gain) to the correct position, then hold there.

The result is that the attitude controller __typically does not require tuning__. If the attitude controller is aggressively tuned, or the system oscillates around a specific attitude with a a period of 1-2Hz (but is stable), then you may consider tuning down the P gain for the attitude controller.

To tune, first ensure that the system response appropriately in rate/Acro mode, then adjust the P gain of the attitude controller in stabilised mode until the response is oscillation-free and appropriately responsive to stick inputs.

#### Relevant Parameters
- `MC_ANGLE_P`: the P gain for the attitude controller. Values of `3.5` to `4.5` are suitable for most systems.
- `MC_YAW_W`: adjusts the amount of "yaw influence" the system will utilise during attitude correction. Values of `0.6` to `0.8` will favour faster roll-pitch manoeuvres over slower yaw manoeuvres to reduce attitude error. A value of `1.0` will still allow the system to operate fine.
- `MAX_ROLL_A` and `MAX_PITCH_A`: Limits the maximum commanded roll and pitch angles __from the RC transmitter during stabilised mode__.

## Resources for Better Understanding the Tuning Process
- [Wikipedia page on PID Controllers (and the effects of each gain)](https://en.wikipedia.org/wiki/PID_controller)
- [PX4 tuning guide (on which the controllers in __robin__ ware based)](https://docs.px4.io/v1.10/en/config_mc/pid_tuning_guide_multicopter.html] --- Note: the tuning parameters are different between the two systems, so values are not directly applicable. Only the actual tuning process is relevant.
