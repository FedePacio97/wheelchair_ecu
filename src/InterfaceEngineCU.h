#ifndef InterfaceEngineCU_h
#define InterfaceEngineCU_h

#include<Arduino.h>
#include <VescUart.h>
#include "messages.h"
//This class provides an abstraction on the protocol we are going to use to talk with the EngineControlUnit.
//In this project we will employ 2 VESC using UART protocol: anyway, you can choose whatever protocol you want
//just provide the correct implementation of the following methods

//Notification values
#define EngineCU_DEAD 0x01
#define dummy_32_bit 0xdeadbeef


class InterfaceEngineCU {
    
    /** Initiate VescUart class */
    VescUart VESC_LX; //relative to LX motor
    VescUart VESC_RX;

    //Set everytime a mex is received, reset by periodical task with period KEEP_ALIVE_PERIOD_ms (apply mutex)
    bool received_message_within_keep_alive_period;
    //To guard "received_message_within_keep_alive_period" since concurrently accesed by checking task(reset) and received mex task(set)
    SemaphoreHandle_t xSemaphoreMutex; //differs from binary semaphores since implements PRIORITY INHERITANCE (reduce the impact of priority inversion problem)
    //Used to check if within that checking period at least a mex has been received from the EngineCU: if not so, something wrong is happening
    //so enable security_stop_procedure
    const int KEEP_ALIVE_PERIOD_ms = 1000;
    const int CONSISTENCY_CHECKING_PERIOD_ms = 2000;

    
  public:
    InterfaceEngineCU(/* variables */);
    //int send_tauLX_tauRX(uint8_t tauLX, uint8_t tauRX); //Returns 1 if correctly send, 0 otherwise
    bool retrieve_motors_info(Telemetry_info_from_VESC&,Telemetry_info_from_VESC&); //ask both vesc about info

    bool set_RPM_motors(int RPM_motor_lx, int RPM_motor_rx);

    void set_CURRENT_motors(float CURRENT_motor_lx, float CURRENT_motor_rx);

    void set_received_message_within_keep_alive_period();
    void reset_received_message_within_keep_alive_period();

    //getter
    int get_KEEP_ALIVE_PERIOD_ms();
    int get_CONSISTENCY_CHECKING_PERIOD_ms();
    bool get_received_message_within_keep_alive_period();

    void initialize_serial_ports_for_UART_communication_with_VESCs();
    void initialize_bluetooth_communication_with_joystick();

  private:
    
    
};

#endif /*InterfaceEngineCU_ECU_h*/