# BM_SenseEsp_V.9

|<img src="https://github.com/user-attachments/assets/6dd8ae96-085b-44be-a9f0-b0a2c5a6a411" width="400" height="300">|<img src="https://github.com/user-attachments/assets/af1330a7-a50f-4b4d-aa2b-fa7107b0a245" width="400" height="300">|
|:-:|:-:|


This repository contains example code for a Battery Monitor based on SensEsp V3.0.0. This is a single shunt battery monitor.

The design supports low side shunt monitoring using a TI INA219. The maximum battery voltage for this configuration is 26V.

Alternativly the design can support low or high side shunt monitoring using a TI INA226. The maximum battery voltage for this configuration is 36V.

This hardware is designed to connect to an NMEA2000 network for power and network communications. 

The base software transmits the follong NMEA2000 PIDs:

PGN 127513 "Battery Configuration Status"

PGN 127508 "Battery Status"

Optionally the hardware/software may also connect with a SignalK server via a wifi. 

The base software transmits the following Signalk paths:

--- TBD ----

This project is a work in progress. Currently the repository contains a functional example 
for a Shunt Monitor using Sensesp V3.

The hardware for this projhect can be found over here:
https://www.tindie.com/products/cnjboards/esp32-dc-power-monitor-with-nmea2000-interface/

Enjoy
