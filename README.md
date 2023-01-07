# Wheelchair ECU
This repository contains code that will be running on ESP32 and is in charge of handling the movment of a wheelchair.
# Needed components
* Microcontroller Unit: ESP32
* IMU: ICM-20948
* Engine Control Unit: 2x VESC 100 A
* Joystick: 2-axis joystick

# System architecture

![System architecture overview](https://drive.google.com/uc?export=view&id=1jS1joBsyRPNTYBAfYW_id2PbQPkl7GWn)

The system is composed of two sub-systems:
* wheelchair_ecu (code contained in this repo)
* wheelchair_joystick_screen

# System Class diagram

![System class diagram](https://drive.google.com/uc?export=view&id=1pjPUHj7t_G1Sb604eiz-txAvEPVVAKz3)

# Physical Implementation

![System class diagram](https://drive.google.com/uc?export=view&id=1IMJsm1ykFDSBgv9yRazRH47dvv3L1iyP)

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

# How to use

Assumption: This code has been developed using Platformio extension on VS Code. Even though you could compile this project using whatever toolchain is compatible with ESP32 hw, it is recommended to priorly install Platformio and set-up its toolchain for ESP32. Reference: https://randomnerdtutorials.com/vs-code-platformio-ide-esp32-esp8266-arduino/

1. Clone this repository
2. Import this project using Platformio wizard
3. Compile the project
4. Flash the compiled project onto an ESP32