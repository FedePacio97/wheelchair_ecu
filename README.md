# Wheelchair ECU
This repository contains code that will be running on ESP32 and is in charge of handling the movment of a wheelchair.
# Needed components
* Microcontroller Unit: ESP32
* IMU: ICM-20948
* Engine Control Unit: 2x VESC 100 A
* Joystick: 2-axis joystick

# System architecture

![System architecture overview](https://drive.google.com/file/d/1jS1joBsyRPNTYBAfYW_id2PbQPkl7GWn/view?usp=share_link)

The system is composed of two sub-systems:
* wheelchair_ecu (code contained in this repo)
* wheelchair_joystick_screen

# System Class diagram

![System class diagram](https://drive.google.com/file/d/1pjPUHj7t_G1Sb604eiz-txAvEPVVAKz3/view?usp=share_link)

# Physical Implementation

![System class diagram](https://drive.google.com/file/d/1IMJsm1ykFDSBgv9yRazRH47dvv3L1iyP/view?usp=share_link)

## Pinout

**SPI**
| **ESP32** | **ICM-20948** |
|-----------|---------------|
|   GPIO33  |       CS      |
|   GPIO19  |      MISO     |
|   GPIO14  |      SCLK     |
|   GPIO23  |      MOSI     |

**UART**

| **ESP32**  | **VESC_right** |
|------------|----------------|
| (RX) GPIO2 |       TX       |
| (TX) GPIO4 |       RX       |

| **ESP32**   | **VESC_right** |
|-------------|----------------|
| (RX) GPIO16 |       TX       |
| (TX) GPIO17 |       RX       |