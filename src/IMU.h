#ifndef IMU_h
#define IMU_h

#include<Arduino.h>
#include <Teensy-ICM-20948.h>
#include <FastRunningMedian.h>

#define WHEELCHAIR_STABLE 0x30
#define WHEELCHAIR_RISK_OVERTURNING_PITCH 0x31
#define WHEELCHAIR_RISK_OVERTURNING_ROLL 0x32
#define WHEELCHAIR_OVERTURNED_ROLL 0x33
#define WHEELCHAIR_OVERTURNED_PITCH 0x34

struct AHSR{
  int pitch,roll,yaw;
};//Attitude and heading reference system

class IMU {

  AHSR ahsr;
  TeensyICM20948 icm20948; //Actual IMU type used
  TeensyICM20948Settings icmSettings;
  //Filters used to get rid of outliers in reading yaw,pitch,roll
  FastRunningMedian<int,8, 0> myMedianRoll;
  FastRunningMedian<int,8, 0> myMedianPitch;
  FastRunningMedian<int,8, 0> myMedianYaw;
  //safety threshold
  const int WHEELCHAIR_RISK_OVERTURNING_ROLL_threshold = 30;
  const int WHEELCHAIR_RISK_OVERTURNING_PITCH_threshold = 30;
  const int WHEELCHAIR_OVERTURNED_ROLL_threshold = 60; //In our case makes no difference in terms of safety action in what direction the wheelchair has overturned: anyway, for future work, it may be interesting to investigate the frequency of them
  const int WHEELCHAIR_OVERTURNED_PITCH_threshold = 60;

  public:
    IMU(/* variables */);
    void updateSensors(); //Update filtered data into ahsr
    uint16_t check_stability(); //Check current orientation (pitch and roll below safe threshold) (The pitch should be checked before since if near 90° effect of Euler angle singularity, i.e roll bursts)
    int get_pitch();
    int get_roll();
    int get_yaw();
    AHSR get_ahsr();
  private:
    
};

#endif /*IMU_h*/