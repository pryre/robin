# Documentation
Here you can find the current list of documentation:
- [Supported Devices](PINOUT.md)
- [Interfacing](INTERFACING.md)
- [Parameters](PARAMETERS.md)
- [Calibration](CALIBRATION.md)
- [Bundled Tools](TOOLS.md)
- [Miscellaneous](OTHER.md)

## TODOs
A listing of supporting documentation for the robin autopilot. This should probably be moved to the wiki, but for the time being...
 - Document what hardware will only work with external power
 - Write a listing of what outputs are what
 - Document methods for adding "Modules" to the codebase
 - Add in descriptions of estimator and controller methods
 - Make a listing about global data structs
 - Explain fix16, v3d, mat16, and qf16 (especially quaternion w,x,y,z -> a,b,c,d)
 - Explain Safety/Mav State Machine
 - How to set up motors, how to set PWM values (esp. PWM_MIN) / idle values (0 to disable idle) / etc.
 - Explain considerations needed when selecting buad rates:
   - We really need to consider how the baud affects Comms
   - A baud rate of 921600 will get us a data rate of 115200 bytes/sec
   - This is assuming we don't use stop bits
   - This means we can send ~115 bytes per loop without any risk of a buffer overflow
   - As mavlink can have max messages of 250+ bytes, we want to send as little as possible each loop to allow catchup
 - Note about how mavesp8266 does not work well in AP mode (too many dropped packets)
