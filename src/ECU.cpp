#include<Arduino.h>
#include"Joystick.h"
#include"ECU.h"

#define DEBUG_LEVEL 2

            //Initialization list
ECU::ECU() : joystick(JOYSTICK_Y_PIN,JOYSTICK_X_PIN) /*, Accelerometer(), [...] */{
  angular_speed_rear_wheels.omega_left = angular_speed_rear_wheels.omega_right = 0;
  wheelchair_reference_speed.angular = wheelchair_reference_speed.linear = 0;
  reference_RPM_rear_wheels.RPM_lx = reference_RPM_rear_wheels.RPM_rx = 0;
  //MAX_RPM = DEFAULT_MAX_RPM;
}

/*
void ECU::update_reference_RPM_rear_wheels(){
  joystick.update_charthesian_coordinates();
  int degree = joystick.get_discretized_degree();
  float module = joystick.get_module();
  
  if(degree >= 0 && degree <=90 ){ //QUADRANT 1 [0,90] 
    reference_RPM_rear_wheels.RPM_rx = module * map(degree, 0, 90, -1 * MAX_RPM, MAX_RPM );
    reference_RPM_rear_wheels.RPM_lx = module * MAX_RPM;
    return;
  }
  if(degree >= 90 && degree <= 180){ //QUADRANT 2 ]90,180]
      reference_RPM_rear_wheels.RPM_rx = module * MAX_RPM;
      reference_RPM_rear_wheels.RPM_lx = module * map(degree,180, 90,-1 * MAX_RPM, MAX_RPM);
      return;
  }
  if(degree >= -180 && degree <= -90){ //QUADRANT 3 [-180,-90]
      reference_RPM_rear_wheels.RPM_rx = module * map(degree,-90, -180,-1 * MAX_RPM, MAX_RPM);
      reference_RPM_rear_wheels.RPM_lx = -1 * module * MAX_RPM;   
      return;
  }
  if(degree > -90 && degree < 0){ //QUADRANT 4 [-90,0[
      reference_RPM_rear_wheels.RPM_rx = -1 * module * MAX_RPM;
      reference_RPM_rear_wheels.RPM_lx = module * map(degree,0, -90,MAX_RPM, -1 * MAX_RPM);
  }
  

}

int ECU::get_reference_RPM_lx(){
  return reference_RPM_rear_wheels.RPM_lx;
}

int ECU::get_reference_RPM_rx(){
  return reference_RPM_rear_wheels.RPM_rx;
}*/

void ECU::update_angular_reference_speed_rear_wheels(){
  joystick.update_charthesian_coordinates();
  int degree = joystick.get_discretized_degree();
  float module = joystick.get_module();
  
  #if DEBUG_LEVEL > 1
    Serial.printf("Joystick module -> %d\t degree -> %f\n",module,degree);
  #endif

  if(degree >= 0 && degree <=90 ){ //QUADRANT 1 [0,90] 
    angular_speed_rear_wheels.omega_right = module * map(degree, 0, 90, -1 * MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED );
    angular_speed_rear_wheels.omega_left = module * MAX_ANGULAR_SPEED;
    return;
  }
  if(degree >= 90 && degree <= 180){ //QUADRANT 2 ]90,180]
      angular_speed_rear_wheels.omega_right = module * MAX_ANGULAR_SPEED;
      angular_speed_rear_wheels.omega_left = module * map(degree,180, 90,-1 * MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
      return;
  }
  if(degree >= -180 && degree <= -90){ //QUADRANT 3 [-180,-90]
      angular_speed_rear_wheels.omega_right = module * map(degree,-90, -180,-1 * MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
      angular_speed_rear_wheels.omega_left = -1 * module * MAX_ANGULAR_SPEED;   
      return;
  }
  if(degree > -90 && degree < 0){ //QUADRANT 4 [-90,0[
      angular_speed_rear_wheels.omega_right = -1 * module * MAX_ANGULAR_SPEED;
      angular_speed_rear_wheels.omega_left = module * map(degree,0, -90,MAX_ANGULAR_SPEED, -1 * MAX_ANGULAR_SPEED);
  }
  

}

//Usage
    //ECU ecu = ECU(..);
    //ecu.update_wheelchair_reference_speed()
    //float reference_linear_speed = ecu.get_reference_linear_speed();
    //float reference_angular_speed = ecu.get_reference_angular_speed();

float ECU::get_reference_angular_speed(){
  return wheelchair_reference_speed.angular;
}

float ECU::get_reference_linear_speed(){
  return wheelchair_reference_speed.linear;
}

int ECU::get_reference_RPM_lx(){
  return reference_RPM_rear_wheels.RPM_lx;
}

int ECU::get_reference_RPM_rx(){
  return reference_RPM_rear_wheels.RPM_rx;
}

void ECU::update_wheelchair_reference_speed(){
  update_angular_reference_speed_rear_wheels(); //Update angular_speed_rear_wheels struct

  float v_ref_without_limit = (angular_speed_rear_wheels.omega_right + angular_speed_rear_wheels.omega_left) * R / 2; 
  float omega_ref_without_limit = (angular_speed_rear_wheels.omega_right - angular_speed_rear_wheels.omega_left) * R / d;

  #if DEBUG_LEVEL > 1
    Serial.printf("omega_ref_without_limit -> %f\n",omega_ref_without_limit);
    Serial.printf("v_ref_without_limit -> %f\n",v_ref_without_limit);
  #endif

  //limit_wheelchair_reference_speed();
  float v_ref_with_limit = mapFloat(v_ref_without_limit, -1 * V_MAX_WITHOUT_LIMIT, V_MAX_WITHOUT_LIMIT,-1 * V_MAX_LIMIT, V_MAX_LIMIT ); // = v_ref_without_limit * V_MAX_LIMIT / V_MAX_WITHOUT_LIMIT
  float omega_ref_with_limit = mapFloat(omega_ref_without_limit,-1 * OMEGA_MAX_WITHOUT_LIMIT, OMEGA_MAX_WITHOUT_LIMIT,-1 * OMEGA_MAX_LIMIT, OMEGA_MAX_LIMIT);

  wheelchair_reference_speed.linear = v_ref_with_limit;
  wheelchair_reference_speed.angular = omega_ref_with_limit;

}

void ECU::update_RPM_reference_rear_wheels(){

  //From wheelchair_reference_speed to angular_reference_speed_rear_wheels
  float omega_right = wheelchair_reference_speed.linear / R + (wheelchair_reference_speed.angular * d) / (2 * R);
  float omega_left = wheelchair_reference_speed.linear / R - (wheelchair_reference_speed.angular * d) / (2 * R);

  //from angular_reference_speed_rear_wheels to reference_RPM_rear_wheels

  //revolutions per minute = radians per second × 9.549297
  reference_RPM_rear_wheels.RPM_lx = omega_left * 9.549297 * SCALE_RPM_FACTOR;
  reference_RPM_rear_wheels.RPM_rx = omega_right * 9.549297 * SCALE_RPM_FACTOR;

}