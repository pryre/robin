# Mixers
Mixer profiles currently mirror those available in the [PX4 Airframe Reference](https://dev.px4.io/en/airframes/airframe_reference.html) (as of April 2018). As the PX4 documentation may change in the future, take care and test motor directions before flight!

The mixer profiles can be found in `./include/mixers/`. Custom configurations require modification to both `./include/mixer.h` and `./src/mixer.c`.

## Supported Mixers
Currently, the following mixers are supported:
| **Mixer Decription** | **PX4 Name** | `SYS_AUTOSTART` |
|:-------------------- |:------------ | ---------------:|
| Disabled | --- | 0 |
| Fixed-Wing Standard Pass-Through | Standard Plane | 2100 |
| Quadrotor X4 | Generic Quadrotor x | 4001 |
| Quadrotor +4 | Generic Quadrotor + | 5001 |
| Hexarotor X6 | Generic Hexarotor x geometry | 6001 |
