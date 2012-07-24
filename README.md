MultiWii Meets HoTT
==============
MultiWii is a flight control system created by [Alexinparis] mainly for MultiRotor systems.
This projects contains an extension for the current MultiWii 2.1 software to send in flight telemetry data, 
and report/change current settings of the flight system, if HoTT v4 capable RxTx systems from [Graupner] are used.
Licensed under [GPLv3].

Caution
------------
Please don't use the extension provided directly from master branch as it is software in development, which could mean that
it has never been tested out in the field. [Tags](MultiWii-HoTT/tags) are tested on a real QuadX out in the field that they are ready to fly.

How It Works
------------
Sensor data are continously gathered by the flight control software. Every time the RX requestes telemetry data corresponding sensor data (e.g. LiPo voltage) are packed into a format
that conforms to the HoTT protocol and is beeing sent to the RX via serial interface (RX0TX0 on non ATMega2560 platforms, RX3TX3 on ATMega2560).
Thereby the MCU acts as a HoTT capable sensor and emulates a General Electric Air Module, VARIO Module, or GPS Module. 
Furthermore, the HoTT textmode capability allows complete customizable text output, this is used to display and change the current [PID settings], as well as debug data
and activated sensors. 

What's Needed
------------
1. HoTT v4 capable RxTx system (Please make sure, that you have the latest official available firmware from Grauper on your transmitter and receiver).
2. Enabled telemetry downlink channel on the receiver
3. RxTx signal cable from MCU's UART Pins to the receiver's telemetry port.
4. Update your MCU with this compiled project (please review config.h before uploading to match your settings).

For detailed information, see [wiki pages](MultiWii-HoTT/wiki).

Configuration
-------------
Config parameters can be found in config.h.

Available Telemetry Data
--------------
* VBAT
* Relative height over ground
* Flight time since copter has been armed
* Direction
* Distance to home
* Number of satelittes
* Longitude/Latitude

For detailed information, see [changes.xml]

Available Settings / Information
--------------
* Roll, Pitch, Yaw, Alt, GPS, Level, and Mag PID values 
* Raw ACC, GYRO, MAG, and BARO values. Activated sensor are beeing highlighted.
* i2c_error and for debug purposes: debug3 and debug4

Supported MCU Hardware
----------------------
ATMega328, ATMega2560

Supported Sensor
----------------
* freeIMU0.4.3
* BMP085
* MS561101BA
* Serial/I2C GPS

Limitations
-----------
* The HoTT protocol has some technichal limitation that prevents from sending telemetry data in a apporiate time frame which does not interfere motor control.
That is the reason why only every 2 seconds one telemetry frame is sent, even if it is requested more often. Which results in stop and go telemetry data on the display.
My recommendation is, that you enable only one telemetry sensor at the time, e.g. Electric Air Module, as the stop and go stops if only one sensor is active.
* On MCU platforms that only have one UART, e.g. Arduino ProMini MultiWiiConf Tool cannot be used when telemetry is activated, 
this means you have to recompile the code and deactivate HoTT telemetry to be able to use MultiWiiConf Tool. 
* Increases cycle time up to 35ms every time when telemetry data are transmitted (but it still [flies]).

[GPLv3]: https://github.com/obayer/MultiWii-HoTT/blob/master/LICENSE.txt
[Alexinparis]: http://www.multiwii.com/
[Graupner]: http://www.graupner.de/
[PID settings]: http://www.youtube.com/watch?v=rItCvYUPo_o
[flies]: http://www.youtube.com/watch?v=8MEnRZlQoGY
[changes.xml]: https://github.com/obayer/MultiWii-HoTT/blob/master/changes.xml
