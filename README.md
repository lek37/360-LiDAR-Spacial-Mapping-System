# 360 Spacial Mapper Using Time-of-Flight

This project feature a device that provide a simple interface scanning device, mainly consist of two push buttons that control the system, along with some relevant embedded devices that are integrated together: 

 ## Highlighted spec

👉 Microcontroller **Texas Instrument MSP-EXP432E401Y** is a core device that control the system operation. Some notice features of this devices includes: 
 - Operates at maximum 120 MHz bus speed. This device operates at 26 MHz.
 - 128 GPIO (General Purpose Input Output) Pins for flexible digital communications.
 - 2 on-board push buttons and 4 on-board LEDs, all GPIO controlled.

👉 The **VL53L1X Time-of-Flight Sensor** (ToF) acts as the main characters to acquires measurement data of the system:
- Be able to measure up to 400 cm (4 m) of distance, at maximum sampling frequency of 30 Hz.
- Operating voltage ($V_{IN}$) of 2.6 V to 5.5 V.
- 16-bit distance reading in millimeters using I2C communication protocol.

👉 The **28BYJ-48 Stepper Motor** has the following features:
- Operating voltage of 5V DC
- 4 phases, with gear ratio of 64:1
- 4 LEDs on the driver for indicating current phase.

👉 Two off-board push buttons has the following features:
- Button 1: Start/Stop button. This button is responsible for start and stop the motor spin.
- Button 2: Home button, for returning to the original position of scan when pressed.
- Additionally, there is also a reset button on the microcontroller to activate this device.

👉 System Communication protocol includes:
- Time-of-Flights (ToF) sensor utilizes I2C protocol to send the measurement data captured to the microcontroller.
- **UART** protocol for transmitting the data from the microcontroller, to **MATLAB** (receiver) for serial data reading, at baud rate of 115200 bps.






