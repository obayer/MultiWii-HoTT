MultiWii Meets HoTT
==============
MultiWii is a flight control system created by [Alexinparis] mainly for MultiRotor systems.
This projects contains an extension for the current MultiWii 2.0 software to send in flight telemetry data, 
and report/change current settings of the flight system, if HoTT v4 capable RxTx systems from [Graupner] are used.
Licensed under [GPLv3].

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
3. RxTx signal cable from MCU's UART Pins to the receiver's telemetry port. Be advised that the MCU
uses 5V TTL, whereas the HoTT receiver needs a 3,3V TTL (LLC is needed).
4. Update your MCU with this compiled project (please review config.h before uploading to match your settings).

Connection 
------------
* MCU(RxTx) \-\-\- LLC \-\-\- \(Ch05\) Graupner GR\-12
* HoTT uses one wire to transmit and receive data which means MCU's Rx and Tx line has to be connected together. From RxTx it goes to a LLC to convert
MCU's 5V level to 3,3V HoTT level and vice versa. From LLC the one wire is connected to Graupners receiver on the telemetry port, 
e.g. channel 05 when using a GR-12.

Configuration
-------------
Config parameters can be found in config.h.

Available Telemetry Data
--------------
* VBAT
* Relative height over ground
* Flight time since copter has been armed

Available Settings 
--------------
* Roll, Pitch, Yaw, Alt, GPS, Level, and Mag PID values 

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
* Increases cycle time up to 35ms every time when telemetry data are transmitted (but it still [flies]). 

[GPLv3]: https://github.com/obayer/MultiWii-HoTT/blob/master/LICENSE.txt
[Alexinparis]: http://www.multiwii.com/
[Graupner]: http://www.graupner.de/
[PID settings]: http://www.youtube.com/watch?v=rItCvYUPo_o
[flies]: http://www.youtube.com/watch?v=8MEnRZlQoGY
