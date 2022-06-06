#include<Arduino.h>
#include"InterfaceEngineCU.h"
#include "WiFi.h" //For BLE communication
#include <esp_now.h>
#include "messages.h"

InterfaceEngineCU::InterfaceEngineCU(){

    //Initialize serial communication: we decide Serial1 attached to VESC_LX and Serial2 to VESC_RX
    //Serial1.begin(115200);
    //Serial2.begin(115200);

    //VESC_LX.setSerialPort(&Serial1);
    //VESC_RX.setSerialPort(&Serial2);
    
    //VESC_LX.getVescValues();
    //VESC_RX.getVescValues();

    //Serial.println(VESC_LX.data.current);
    //Serial.println(UART.data.inpVoltage);
    //Serial.println(UART.data.ampHours);
    //Serial.println(UART.data.tachometerAbs);

    //Serial.println(VESC_RX.data.current);
    

    /* Create a mutex type semaphore. */
    xSemaphoreMutex = xSemaphoreCreateMutex();
    if( xSemaphoreMutex == NULL )
    {
        Serial.println("Not enough space for creating a mutex!");
        //Send buzzer alarm!
        while (1);
    }


}

//Used to signal the presence of current motor values
extern TaskHandle_t xHandlerOfTaskCheckConsistencyAmongRPM_IMUvelocity;

//For debugging purpose
#define VESC_RX_LED 26
#define VESC_LX_LED 25

bool InterfaceEngineCU::retrieve_motors_info(Telemetry_info_from_VESC &VESC_LX_info, Telemetry_info_from_VESC &VESC_RX_info){
    bool received_info_lx = false, received_info_rx = false;
    received_info_lx = VESC_LX.getVescValues();
    received_info_rx = VESC_RX.getVescValues();

    if ( received_info_lx && received_info_rx ) {

        //Signal the task which performs the consistency check between RPM and CURRENT IMU VELOCITY
        //xTaskNotify( xHandlerOfTaskCheckConsistencyAmongRPM_IMUvelocity, dummy_32_bit, eSetValueWithOverwrite  );
        
            //Serial.println(VESC_LX.data.current);

            //Serial.println(VESC_RX.data.current);
        //Set the variable stating EngineCU is still alive
            //set_received_message_within_keep_alive_period();

        memcpy(&VESC_LX_info, &VESC_LX.data, sizeof(VESC_LX_info));
        memcpy(&VESC_RX_info, &VESC_RX.data, sizeof(VESC_RX_info));

    }

    if(!received_info_lx){
        #if DEBUG_LEVEL > 1
        Serial.println("Failed to get data from VESC_LX!");
        #endif
        digitalWrite(VESC_LX_LED, LOW);
    }  
    else
        digitalWrite(VESC_LX_LED, HIGH);
        
    if(!received_info_rx){
        #if DEBUG_LEVEL > 1
        Serial.println("Failed to get data from VESC_RX!");
        #endif
        digitalWrite(VESC_RX_LED, LOW);
    }  
    else
        digitalWrite(VESC_RX_LED, HIGH);

    return received_info_lx & received_info_rx;
}

bool InterfaceEngineCU::set_RPM_motors(int RPM_motor_lx, int RPM_motor_rx){

    bool ok_lx = VESC_LX.setRPM(RPM_motor_lx);
    // if(!ok){
    //     Something is wrong.. it will be detected by the watchdog on EngineCU aliveness
    //     return false;
    // }

    bool ok_rx = VESC_RX.setRPM(RPM_motor_rx);

    return ok_lx & ok_rx;
}

void InterfaceEngineCU::set_CURRENT_motors(float CURRENT_motor_lx, float CURRENT_motor_rx){

    VESC_LX.setCurrent(CURRENT_motor_lx);

    VESC_RX.setCurrent(CURRENT_motor_rx);

    return;

}

void InterfaceEngineCU::set_received_message_within_keep_alive_period(){
    xSemaphoreTake( xSemaphoreMutex, portMAX_DELAY);
    received_message_within_keep_alive_period = true;
    xSemaphoreGive( xSemaphoreMutex );
}

void InterfaceEngineCU::reset_received_message_within_keep_alive_period(){
    xSemaphoreTake( xSemaphoreMutex, portMAX_DELAY); //wait undefinetly
    received_message_within_keep_alive_period = false;
    xSemaphoreGive( xSemaphoreMutex );
}

bool InterfaceEngineCU::get_received_message_within_keep_alive_period(){
    return received_message_within_keep_alive_period;
}

int InterfaceEngineCU::get_KEEP_ALIVE_PERIOD_ms(){
    return KEEP_ALIVE_PERIOD_ms;
}

int InterfaceEngineCU::get_CONSISTENCY_CHECKING_PERIOD_ms(){
    return CONSISTENCY_CHECKING_PERIOD_ms;
}


void InterfaceEngineCU::initialize_serial_ports_for_UART_communication_with_VESCs(){
    /** Setup UART port for communication with VESCs */
    Serial1.begin(115200);
    Serial2.begin(115200);

    VESC_RX.setSerialPort(&Serial1);
    VESC_LX.setSerialPort(&Serial2);
}

void InterfaceEngineCU::initialize_bluetooth_communication_with_joystick(){
    WiFi.mode(WIFI_MODE_STA);
    delay(10);
    #if DEBUG_LEVEL > 1
    Serial.println(WiFi.macAddress());
    #endif
    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);
    
    // Register peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, joystick_controller_MAC, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
    }
    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(OnDataRecvEngineCUController);
}