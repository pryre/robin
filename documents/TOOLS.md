# Tools
[Back to index](README.md).

## QGroundControl
Currently, QGCS support is patchy at best. Most of the calibration and management functionallity is available using the [custom widget](https://github.com/qutas/robin/blob/master/lib/qgroundcontrol_plugins/RobinCommandPanel.qml), which can be loaded in QGCS by clicking "Widgets > Custom Command > Load Custom QML file".

It is currently compatible to do the following:
- Allow you to select and change parameters
- Airframe selection for those that are supported, but cannot be applied corectly in the Airframe Menu
- Heads up, maps, and virutal horizon work as expected
- Parameters can be saved to EEPROM to persist to the next boot with the [widget](https://github.com/qutas/robin/blob/master/lib/qgroundcontrol_plugins/RobinCommandPanel.qml)
- Calibrations technically work, but only through the [widget](https://github.com/qutas/robin/blob/master/lib/qgroundcontrol_plugins/RobinCommandPanel.qml)
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
