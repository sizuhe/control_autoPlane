`autoPlane` PID system made for providing flight stabilization and automatic flight capabilities to an RC plane, these capabilities allow the plane to maintain a stable flight path and follow a specified course angle set by the user. This system can be calibrated and configured through Bluetooth communication and sends telemetry data to a ground station using a LoRa module.



# Hardware
- ESP32
- LoRa SX1278
- MPU9250
- BMP180
- 2 Servos (pitch and roll)



# Components
| **Battery** | **MCU (e.g. ESP32)** | **LoRa SX1278** | **MPU9250** | **BMP180** | **Servo (Pitch)** | **Servo (Roll)** |
|:-----------:|:--------------------:|:---------------:|:-----------:|:----------:|:-----------------:|:----------------:|
|    11.1 V   |          VIN         |        -        |      -      |      -     |         -         |         -        |
|    5.0 V    |           -          |        -        |      -      |      -     |        VCC        |        VCC       |
|    3.3 V    |           -          |       VCC       |     VCC     |     VCC    |         -         |         -        |
|     GND     |          GND         |       GND       |     GND     |     GND    |        GND        |        GND       |
|      -      |          D4          |       DI0       |      -      |      -     |         -         |         -        |
|      -      |          D16         |        -        |     SCL     |     SCL    |         -         |         -        |
|      -      |          D17         |        -        |     SDA     |     SDA    |         -         |         -        |
|      -      |          D19         |        -        |      -      |      -     |         -         |        PWM       |
|      -      |          D21         |        -        |      -      |      -     |        PWM        |         -        |
|      -      |          D25         |       MOSI      |      -      |      -     |         -         |         -        |
|      -      |          D26         |        CS       |      -      |      -     |         -         |         -        |
|      -      |          D27         |       RST       |      -      |      -     |         -         |         -        |
|      -      |          D32         |       SCK       |      -      |      -     |         -         |         -        |
|      -      |          D33         |       MISO      |      -      |      -     |         -         |         -        |



# Notes
- RC plane used a V-tail configuration, therefore there's not direct yaw control nor servos used for this movement.
- Bluetooth communication can be used for magnetometer calibration, data logging or arming/disarming servos.
- This code has capabilities to add a buzzer and voltage and current sensors.
- Constant variables are stored in a struct data type to differentiate between various uses of constants.



# Commands
When using Bluetooth communication some functions can be executed with the following input values:

- **10**: Magnetometer and IMU calibration.
- **20**: Disarms servos.
- **21**: Arms servos.