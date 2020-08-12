# Tuning
[Back to index](README.md).

## Pre-Tuning Notes

## Understanding the Gains

## Rate Controllers

## Attitue Controller
The attitude controller follows a much simplier process as it is only a proportional controller. The theory being that if the rate response is good, then commanding "hold-still at this attitude" will give the intended response, thus we only need to "guide" the attitude (with the P gain) to the corect position, then hold there.

The result is that the attitude controller __typically does not require tuning__. If the attitude controller is agressively tuned, or the system oscilates around a specific attitude with a a period of 1-2Hz (but is stable), then you may consider tuning down the P gain for the attitude controller.

#### Relevant Parameters
- `MC_ANGLE_P`: the P gain for the attitude controller. Values of `3.5` to `4.5` are suitable for most systems.
- `MC_YAW_W`: adjusts the amount of "yaw influence" the system will utilise during attitude correction. Values of `0.6` to `0.8` will favour faster roll-pitch maneuvers over slower yaw maneuvers to reduce attitude error. A value of `1.0` will still allow the system to operate fine.
- `MAX_ROLL_A` and `MAX_PITCH_A`: Limits the maximum commanded roll and pitch angles __from the RC transmitter during stabilised mode__.

## Resources for Better Understanding the Tuning Process
- [Wikipedia page on PID Controllers (and the effects of each gain)](https://en.wikipedia.org/wiki/PID_controller)
-

