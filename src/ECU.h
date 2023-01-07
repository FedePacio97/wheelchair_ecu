#ifndef ECU_h
#define ECU_h

#include<Arduino.h>
#include"Joystick.h"

//GPIO 4 and 2 Potentiometer
const gpio_num_t JOYSTICK_Y_PIN = GPIO_NUM_34;
const gpio_num_t JOYSTICK_X_PIN = GPIO_NUM_35;


const float d = 0.2; //distance between wheels

const int MAX_ANGULAR_SPEED = 100; //Config param
const float V_MAX_WITHOUT_LIMIT = 40; //m/s JUST NUMBERS GOT BY EQUATIONS.. NOT REALLY MEANINGFUL BUT USED TO SCALE
const float OMEGA_MAX_WITHOUT_LIMIT = 400; //rad/s JUST NUMBERS GOT BY EQUATIONS.. NOT REALLY MEANINGFUL BUT USED TO SCALE
const float V_MAX_LIMIT = 12; //m/s 
const float OMEGA_MAX_LIMIT = 3; //rad/s

const int DEFAULT_MAX_RPM = 1000; //Config param
const int SCALE_RPM_FACTOR = 100; //For scaling up RPM from omega references

const int TACHOMETER_MOTOR_PER_WHEEL_ROUND = 400; //every 400 units -> 1 round of wheel
const float R = 0.59; //wheel radius

struct Angular_speed_rear_wheels{
    float omega_right,omega_left;
};

struct Wheelchair_reference_speed{ //Reference speed of baricenter G (linear [m/s], angular [rad/s])
    float angular,linear;
};

struct Reference_RPM_rear_wheels{
  int RPM_lx,RPM_rx;
};


class ECU {
    
    Joystick joystick;

    //To be given to VESCs
    Reference_RPM_rear_wheels reference_RPM_rear_wheels;
    //int MAX_RPM;
    
    Angular_speed_rear_wheels angular_speed_rear_wheels;
    //Velocity (linear,angular)
    Wheelchair_reference_speed wheelchair_reference_speed;
  public:
    ECU(/* variables */);
    void update_wheelchair_reference_speed();
    void update_RPM_reference_rear_wheels();
    int get_reference_RPM_lx();
    int get_reference_RPM_rx();
    float get_reference_linear_speed();
    float get_reference_angular_speed();
  private:
    void update_angular_reference_speed_rear_wheels();
};

#endif /*ECU_h*/