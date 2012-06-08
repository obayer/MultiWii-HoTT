MultiWii Meets HoTT
==============
MultiWii is a flight control system created by [Alexinparis] mainly for MultiRotor systems.
This projects contains an extension for the current MultiWii 2.0 software to send in flight telemetry data, 
and report/change current settings of the flight system, if HoTT v4 capable RxTx systems from [Graupner] are used.
Licensed under [GPLv3].

Caution
------------
Please don't use the extension provided directly from master branch as it is software in development, which could mean that
it has never been tested out in the field. Tags are tested on a real QuadX out in the field that they are ready to fly.

How It Works
------------
Sensor data are continously gathered by the flight control software. Every 2 seconds selected sensor data (e.g. LiPo voltage) are packed into a format
that conforms to the HoTT protocol specifications and is beeing sent to the receiver via MCUs serial interface (RX0TX0 on non Mega2560 platforms, RX3TX3 on Mega2560).
Thereby the MCU acts as a HoTT capable sensor and emulates a General Electric Air Module. 
Furthermore, the HoTT textmode capability allows complete customizable text output, this is used to display and change the current [PID settings]. 

What's Needed
------------
1. HoTT v4 capable RxTx system
2. Enabled telemetry downlink channel on the receiver
3. RxTx signal cable from MCU's UART Pins to the receiver's telemetry port.
4. Update your MCU with this compiled project (please review config.h before uploading to match your settings).

For detailed information see the [wiki pages](MultiWii-HoTT/wiki).

Configuration
-------------
Config parameters can be found in config.h.

Available Telemetry Data
--------------
* VBAT
* Relative height over ground
* Flight time since copter has been armed
* Direction

Available Settings / Information
--------------
* Roll, Pitch, Yaw, Alt, GPS, Level, and Mag PID values 
* Raw ACC, GYRO, MAG values. Getting heighleighted if sensor is activated.
* i2c_error and for debug purposes: debug3 and debug4

Supported MCU Hardware
----------------------
Arduino ProMini, Uno, Mega2560

Supported Sensor
----------------
* freeIMU0.4.3
* BMP085
* MS561101BA

Limitations
-----------
* On MCU platforms that only have one UART, e.g. Arduino ProMini MultiWiiConf Tool cannot be used when telemetry is activated, 
this means you have to recompile the code and deactivate HoTT telemetry to be able to use MultiWiiConf Tool. 
* Increases cycle time up to 35ms every time when telemetry data are transmitted (but it still [flies]). That's the reason, why
telemetry data are updated with 0.5Hz.

[GPLv3]: https://github.com/obayer/MultiWii-HoTT/blob/master/LICENSE.txt
[Alexinparis]: http://www.multiwii.com/
[Graupner]: http://www.graupner.de/
[PID settings]: http://www.youtube.com/watch?v=rItCvYUPo_o
[flies]: http://www.youtube.com/watch?v=8MEnRZlQoGY
