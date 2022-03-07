#ifndef Messages_h
#define Messages_h

#include <Arduino.h>
#include <esp_now.h>

//Uncomment the right one depending on the controller
#define wheelchair_joystick_screen_controller 1
#define wheelchair_engineCU_IMU_controller 1

#ifdef wheelchair_engineCU_IMU_controller
extern QueueHandle_t xRPM_From_Joystick_Queue;
#endif

#ifdef wheelchair_joystick_screen_controller
extern QueueHandle_t xTelemetry_From_ECU_Queue;
extern QueueHandle_t xStability_Info_From_ECU_Queue;
#endif


//List of messages exchanged between wheelchair_joystick_screen controller and wheelchair_engineCU_IMU controller
// REPLACE WITH THE MAC Address of your receiver 
extern uint8_t engineCU_controller_MAC[];// = {0x7C, 0x9E, 0xBD, 0xF3, 0xB9, 0x10};
extern uint8_t joystick_controller_MAC[];// = {0x7C, 0x9E, 0xBD, 0x39, 0x25, 0x64};

//from engineCU to joystick
#define SCREEN_DATA_OPCODE 1

//from joystick to engineCU
#define RPM_REFERENCE_OPCODE 10 // RPM_LX | RPM_RX

//from engineCU to joystick
#define TELEMETRY_OPCODE 20 // VESC_SIDE [LX,RX] | TelemetryInfo telemetry_info
#define VESC_LX_ID 1
#define VESC_RX_ID 2

//from engineCU to joystick
#define STABILITY_INFO_OPCODE 30

struct Telemetry_info_from_VESC{
  float tempFET;
  float tempMotor;
  float avgMotorCurrent;
  float avgInputCurrent;
  float dutyCycleNow;
  long rpm;
  float inpVoltage;
  float ampHours;
  float ampHoursCharged;
  long tachometer;
  long tachometerAbs;
};

struct Message_sent_on_BLE {
    uint8_t OPCODE;
    int8_t* PAYLOAD;
};

struct Message_received_on_BLE {
    uint8_t OPCODE;
    int8_t PAYLOAD[100];
};

struct RPM_message {
    int RPM_LX;
    int RPM_RX;
};

struct RPM_message_sent_on_BLE{
    uint8_t OPCODE;
    RPM_message rpm;
};

struct Stability_message{
    int pitch;
    int roll;
};

struct Stability_message_sent_on_BLE{
    uint8_t OPCODE;
    Stability_message stability_info;
};

struct Telemetry_message{
    float km;
    int speed;
    int battery_level;
    int remaining_km;
};

struct Telemetry_message_sent_on_BLE{
    uint8_t OPCODE;
    Telemetry_message telemetry_message;
};

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);



void OnDataRecvJoystickController(const uint8_t * mac, const uint8_t *incomingData, int len);


//Note that this callback function runs from the WiFi task, which means we should not do lengthy operations on it. 
//For this reason, we just signal the receiver putting the mex in his queue
void OnDataRecvEngineCUController(const uint8_t * mac, const uint8_t *incomingData, int len);
#endif //Messages_h
