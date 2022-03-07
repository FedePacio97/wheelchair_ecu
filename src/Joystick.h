#ifndef Joystick_h
#define Joystick_h

#include<Arduino.h>
#include "FastRunningMedian.h"

#define QUADRANT_1 1
#define QUADRANT_2 2
#define QUADRANT_3 3
#define QUADRANT_4 4


struct Carthesian_coordinates{
    float x,y;
};

class Joystick {
    //Used to discard little variation in potentiometer values due to noise
    unsigned int MINIMUM_VARIATION;
    //Used to discard not clear intention to move
    float ZERO_THRESHOLD;
    //Potentiometer parameters
    const unsigned int ABSOLUTE_MAX_Y = 2468;
    const unsigned int ABSOLUTE_MIN_Y = 1340;
    const unsigned int ABSOLUTE_MAX_X = 2480;
    const unsigned int ABSOLUTE_MIN_X = 1371;
    //Due to correction, it may happend that even if the joystick is put at max, the modulo does not reach 1.
    //Hence, over MAX_MODULO_LOWERBOUND, we consider modulo = 1.
    const float MAX_MODULO_LOWERBOUND = 0.95;
    //To which GPIO the Potentiometer is attached to
    gpio_num_t joystick_Y_pin,joystick_X_pin;
    //Used to store potentiometer readings
    int potentiometer_value_Y,potentiometer_value_X;
    FastRunningMedian<int,16, 0> median_potentiometer_X; //Used to smooth potentiometer/ADC noise
    FastRunningMedian<int,16, 0> median_potentiometer_Y;
    Carthesian_coordinates actual_coordinates;
  public:
    Joystick(gpio_num_t Y_pin,gpio_num_t X_pin,unsigned int minimum_variation = 50, float zero_threshold = 0.15);
    void update_charthesian_coordinates();
    float get_module();
    unsigned short get_quadrant();
    int get_discretized_degree(); //Discritize angle for correction due to small potentiometer granularity
    float get_Y_value();
    float get_X_value();
    int get_Y_potentiometer_value();
    int get_X_potentiometer_value();

    void calibrate_joystick();
  private:
    void sample_potentiometer_values();
    void apply_median_filter(int,int,int&, int&);
    void normalize_carthesian_coordinates();
    void apply_zero_filter();
    double get_degree();
};

float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh);

#endif /*Joystick_h*/