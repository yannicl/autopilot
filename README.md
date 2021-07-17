# autopilot

esp32 based autopilot for a rc boat. This autopilot assists navigation by maintaining course.

In the future, I'll like to develop waypoint navigation using on-board gps receiver.

## board layout (using ESP32-WROOM-32)

## Left side
- 1: 3.3V - Vdd on MPU9250
- 2:
- 3:
- 4:
- 5:
- 6:
- 7:
- 8:
- 9:
- 10:
- 11:
- 12:
- 13:
- 14:
- 15:
- 16:
- 17:
- 18:
- 19: Vin 5V - 5Vcc on RC Receiver (using Hitec Flysky)

## Right side
- 38: GND - GND on RC Receiver - GND on MPU9250
- 37:
- 36: I2C SCL - SCL on MPU9250
- 35:
- 34:
- 33: I2C SDA - SDA on MPU9250
- 32:
- 31:
- 30: GPS Tx
- 29: GPS Rx
- 28:
- 27: Rx2 - FlySky iBus servo output
- 26: Rudder servo (command signal)
- 25:
- 24:
- 23: Front led (command signal)
- 22:
- 21:
- 20:

