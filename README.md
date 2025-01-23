# BM_SenseEsp_V.9
<img src="https://github.com/user-attachments/assets/ace8fbad-32b9-46ae-b1b9-adec5f85cd94" width="450" height="350">

This repository contains example code for a Battery Monitor based on SensEsp V3.0.0. This is a single shunt battery monitor.

The design supports low side shunt monitoring using a TI INA219. The maximum battery voltage for this configuration is 26V.

Alternativly the design can support low or high side shunt monitoring using a TI INA226. The maximum battery voltage for this configuration is 36V.

This hardware is designed to connect to an NMEA2000 network for power and network communications. 

The base software transmits the follong NMEA2000 PIDs:

--- TBD ----

Optionally the hardware/software may also connect with a SignalK server via a wifi. 

The base software transmits the following Signalk paths:

--- TBD ----

It is not necessarily a finished project but is a fully functional example.
