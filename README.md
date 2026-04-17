# 360 Spacial Mapper Using Time-of-Flight

This project feature a device that provide a simple interface scanning device, mainly consist of two push buttons that control the system, along with some relevant embedded devices that are integrated together: 

 ## Highlighted spec

👉 Microcontroller **Texas Instrument MSP-EXP432E401Y** is a core device that communicate the distance measured for data visualization. Some notice features of this devices includes: 
 - Operates at maximum 120 MHz bus speed. This device operates at 26 MHz.
 - 128 GPIO (General Purpose Input Output) Pins for flexible digital communications.
 - 2 on-board push buttons and 4 on-board LEDs, all GPIO controlled.

👉 The **VL53L1X Time-of-Flight Sensor** (ToF) acts as the main characters to acquires measurement data of the system:
- Be able to measure up to 400 cm (4 m) of distance, at maximum sampling frequency of 30 Hz.
- Operating voltage ($V_{IN}$) of 2.6 V to 5.5 V.
- 16-bit distance reading in millimeters using I2C communication protocol.


