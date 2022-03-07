#include "messages.h"

#define DEBUG_LEVEL 0

uint8_t engineCU_controller_MAC[] = {0x7C, 0x9E, 0xBD, 0xF3, 0xB9, 0x10};
uint8_t joystick_controller_MAC[] = {0x7C, 0x9E, 0xBD, 0x39, 0x25, 0x64};

uint8_t received_OPCODE;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  #if DEBUG_LEVEL > 1
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  #endif
}

void OnDataRecvJoystickController(const uint8_t * mac, const uint8_t *incomingData, int len) {
    
    //read opcode
    memcpy(&received_OPCODE, incomingData, sizeof(uint8_t));

    #if DEBUG_LEVEL > 1
    Serial.printf("[MEX RECEIVED] %d\n",received_OPCODE);
    #endif
    
    if(received_OPCODE == TELEMETRY_OPCODE){
        //extract info
        Telemetry_message_sent_on_BLE* telemetry_message_on_BLE = (Telemetry_message_sent_on_BLE*) incomingData;

        Telemetry_message telemetry_message;
        telemetry_message = telemetry_message_on_BLE->telemetry_message;

        //send to queue handled by TaskHandleMotorsParametersFromECU
        xQueueSend( // The handle of the queue.
               xTelemetry_From_ECU_Queue,
               // The address of the xMessage variable.  sizeof( struct AMessage )
               //bytes are copied from here into the queue.
               ( void * ) &telemetry_message,
               // Block time of 0 says don't block if the queue is already full.
               //Check the value returned by xQueueSend() to know if the message
               //was sent to the queue successfully.
               ( TickType_t ) 0 );
    }

    if(received_OPCODE == STABILITY_INFO_OPCODE){
        //extract info
        Stability_message_sent_on_BLE* telemetry_message_on_BLE = (Stability_message_sent_on_BLE*) incomingData;

        Stability_message stability_info;
        stability_info = telemetry_message_on_BLE->stability_info;

        //send to queue handled by TaskHandleMotorsParametersFromECU
        xQueueSend( // The handle of the queue.
               xStability_Info_From_ECU_Queue,
               // The address of the xMessage variable.  sizeof( struct AMessage )
               //bytes are copied from here into the queue.
               ( void * ) &stability_info,
               // Block time of 0 says don't block if the queue is already full.
               //Check the value returned by xQueueSend() to know if the message
               //was sent to the queue successfully.
               ( TickType_t ) 0 );
    }

}

//Note that this callback function runs from the WiFi task, which means we should not do lengthy operations on it. 
//For this reason, we just signal the receiver putting the mex in his queue
void OnDataRecvEngineCUController(const uint8_t * mac, const uint8_t *incomingData, int len) {
    //read opcode
    Serial.printf("Received opcode %d\n",received_OPCODE);
    memcpy(&received_OPCODE, incomingData, sizeof(uint8_t));
    
    if(received_OPCODE == RPM_REFERENCE_OPCODE){
        //extract info
        RPM_message_sent_on_BLE* rpm_message_on_BLE = (RPM_message_sent_on_BLE*) incomingData;

        RPM_message rpm_message_received;
        rpm_message_received.RPM_LX = rpm_message_on_BLE->rpm.RPM_LX;
        rpm_message_received.RPM_RX = rpm_message_on_BLE->rpm.RPM_RX;
        //send to queue handled by TaskWheelchairMovement
        xQueueSend( // The handle of the queue.
               xRPM_From_Joystick_Queue,
               // The address of the xMessage variable.  sizeof( struct AMessage )
               //bytes are copied from here into the queue.
               ( void * ) &rpm_message_received,
               // Block time of 0 says don't block if the queue is already full.
               //Check the value returned by xQueueSend() to know if the message
               //was sent to the queue successfully.
               ( TickType_t ) 0 );
    }
}