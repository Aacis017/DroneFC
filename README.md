# DroneFC - Arduino Flight Controller

A complete, modular flight controller for Arduino Nano IoT 33.

## Features

- **6-DOF IMU** using built-in LSM6DS3 with complementary filter
- **Barometer** BMP180 for altitude hold
- **PID Control** for roll, pitch, yaw, and altitude
- **UART Protocol** JSON command interface for Flask server
- **Motor Mixing** for quadcopter X configuration
- **Safety** arming checks and failsafes

## Hardware Requirements

- Arduino Nano IoT 33
- BMP180 barometer (I2C)
- 4x Brushless ESCs (standard 1000-2000μs PWM)
- 4x Brushless motors
- Raspberry Pi Zero 2W (for Flask server control)

## Pin Configuration

| Component | Pin |
|-----------|-----|
| Motor FL  | 2   |
| Motor FR  | 5   |
| Motor RL  | 3   |
| Motor RR  | 9   |
| BMP180 SDA| A4  |
| BMP180 SCL| A5  |
| UART TX   | TX  |
| UART RX   | RX  |

## Required Libraries

Install these via Arduino Library Manager:

- `Adafruit BMP085 Library`
- `Arduino_LSM6DS3`
- `ArduinoJson`

## Command Protocol

Send JSON commands via Serial1 (UART from Raspberry Pi):

```json
{"cmd":"arm"}
{"cmd":"disarm"}
{"cmd":"throttle","val":1500}
{"cmd":"move","roll":0,"pitch":10,"yaw":5}
{"cmd":"altitude","val":2.0}
{"cmd":"takeoff","alt":1.5}
{"cmd":"land"}
{"cmd":"mode","mode":"althold"}
{"cmd":"status"}
```

## Telemetry Response

```json
{"alt":150,"roll":2,"pitch":-1,"yaw":45,"armed":true,"mode":0,"thr":1500}
```

## PID Tuning

Edit `config.h` to adjust PID gains:

```cpp
#define PID_ROLL_P      1.5f
#define PID_ROLL_I      0.02f
#define PID_ROLL_D      12.0f
```

Start with P only, then add D for stability, finally add I to eliminate steady-state error.

## Safety Notes

⚠️ **ALWAYS remove propellers during initial testing!**

1. Verify ESC calibration before flight
2. Test motor direction (FL/RR should spin CCW, FR/RL should spin CW)
3. Start with low PID gains
4. Test in a safe, open area

## File Structure

```
DroneFC/
├── DroneFC.ino     # Main sketch
├── config.h        # Configuration
├── types.h         # Data structures
├── pid.h           # PID controller
├── motors.h        # Motor control
├── sensors.h       # BMP180 driver
├── imu.h           # IMU processing
├── command.h       # UART protocol
└── README.md       # This file
```
