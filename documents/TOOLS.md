# Tools

## QGroundControl
Currently, QGCS support is patchy at best. It is currently compatible to do the following:
- Allow you to select and change parameters
- Airframe selection for those that are supported, but cannot be applied corectly in the Airframe Menu
- Heads up, maps, and virutal horizon work as expected
- Parameters can only be saved to EEPROM by using a custom tool to call the appropriate command: [widget](lib/qgroundcontrol_plugin/RobinCommandWidget.qml)
- Calibrations technically work
  - Gyro just place on a flat surface and click button once
  - Accel works by clicking the button over and over and following instructions

## rqt_robin_gcs
An RQT app is available to assist with:
- Calibration
- Writing params to eeprom
- Rebooting:
  - System
  - Bootloader

It can be found under the QUTAS repositories as [rqt_robin_gcs](https://github.com/qutas/rqt_robin_gcs)