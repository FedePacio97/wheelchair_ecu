#include<Arduino.h>
#include"Joystick.h"

#define DEBUG_LEVEL 0

//Usage
    //Joystick joystick = Joystick(..);
    //joystick.update_charthesian_coordinates()
    //int degree = joystick.get_discretized_degree();
    //float module = joystick.get_module();

int Joystick::get_discretized_degree(){
    double degree = get_degree();

    int discretized_degree =  (int)degree/10 * 10; //like flooring
    if(abs(discretized_degree) == 170) discretized_degree = 180; //like ceiling
    
    #if DEBUG_LEVEL > 1
    Serial.print("Discretized degree -> ");
    Serial.println(discretized_degree);
    #endif

    return discretized_degree;
}

double Joystick::get_degree(){
    //Handle exception of zero coordinate
    if(actual_coordinates.y == 0 && actual_coordinates.x >= 0) return 0;
    if(actual_coordinates.y == 0 && actual_coordinates.x < 0) return 180;
    if(actual_coordinates.x == 0 && actual_coordinates.y >= 0) return 90;
    if(actual_coordinates.x == 0 && actual_coordinates.y < 0) return -90;
    

    double rad = atan2(actual_coordinates.y, actual_coordinates.x);
    double deg = rad * (180 / PI);
    //Discritize angle for correction due to small potentiometer granularity
    #if DEBUG_LEVEL > 1
    Serial.print("Degree -> ");
    Serial.println(deg);
    #endif

    return deg;
}


unsigned short Joystick::get_quadrant(){
    if(actual_coordinates.x >= 0 && actual_coordinates.y >= 0)
        return QUADRANT_1;
    if(actual_coordinates.x >= 0 && actual_coordinates.y < 0)
        return QUADRANT_4;
    if(actual_coordinates.x < 0 && actual_coordinates.y < 0)
        return QUADRANT_3;
    //if(X < 0 && Y >= 0)
    return QUADRANT_2;
}


float Joystick::get_module(){
    float module = sqrt(actual_coordinates.y * actual_coordinates.y + actual_coordinates.x * actual_coordinates.x);
    //Normalize since potentiometer is not a perfect circle
    module = constrain(module, 0, 1);

    #if DEBUG_LEVEL > 1
    Serial.printf("Modulo before correction -> %f\n", module);
    #endif
    
    if(module > MAX_MODULO_LOWERBOUND) module = 1;
    
    return module;
}

float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh){
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow; 
}

void Joystick::apply_zero_filter (){
    if(abs(actual_coordinates.y) < ZERO_THRESHOLD)
        actual_coordinates.y = 0;

    if(abs(actual_coordinates.x) < ZERO_THRESHOLD)
        actual_coordinates.x = 0;

    #if DEBUG_LEVEL > 1
    Serial.printf("Coordinates after zero_filtering -> X %f \t Y %f\n", actual_coordinates.x,actual_coordinates.y);
    Serial.printf("ZERO_THRESHOLD -> %f\n", ZERO_THRESHOLD);
    #endif
}

void Joystick::normalize_carthesian_coordinates (){
    actual_coordinates.y = mapFloat(potentiometer_value_Y, ABSOLUTE_MIN_Y, ABSOLUTE_MAX_Y, - 1, 1);
    actual_coordinates.x = mapFloat(potentiometer_value_X, ABSOLUTE_MIN_X, ABSOLUTE_MAX_X, - 1, 1);

    #if DEBUG_LEVEL > 1
    Serial.printf("Coordinates -> X %f \t Y %f\n", actual_coordinates.x,actual_coordinates.y);
    #endif
}

void Joystick::apply_median_filter(int read_x,int read_y,int& filtered_x, int& filtered_y){

    //REMOVE NOISE
    median_potentiometer_X.addValue(read_x);
    median_potentiometer_Y.addValue(read_y);

    filtered_x = median_potentiometer_X.getMedian(); // retieves the median
    filtered_y = median_potentiometer_Y.getMedian();
}

void Joystick::sample_potentiometer_values(){
    /*
    int current_potentiometer_value_Y = GPIO_INPUT_GET(joystick_Y_pin);
    int current_potentiometer_value_X = GPIO_INPUT_GET(joystick_X_pin);
    */

    /*int current_potentiometer_value_Y = analogRead(joystick_Y_pin);
    int current_potentiometer_value_X = analogRead(joystick_X_pin);*/

    
    int read_x = analogRead(joystick_X_pin);
    int read_y = analogRead(joystick_Y_pin);

    #if DEBUG_LEVEL > 1
    Serial.printf("read_x X %d \t read_Y %d\n",read_x,read_y);
    #endif

    /*
    //REMOVE NOISE
    median_potentiometer_X.addValue(read_x);
    median_potentiometer_Y.addValue(read_y);

    int current_potentiometer_value_X = median_potentiometer_X.getMedian(); // retieves the median
    int current_potentiometer_value_Y = median_potentiometer_Y.getMedian(); // retieves the median
    */

    
    int current_potentiometer_value_X,current_potentiometer_value_Y;
    apply_median_filter(read_x,read_y,current_potentiometer_value_X,current_potentiometer_value_Y);

    #if DEBUG_LEVEL > 1
    Serial.printf("Potentiometer X %d \t Y %d\n",current_potentiometer_value_X,current_potentiometer_value_Y);
    #endif

    //Include DEADZONE correction
    if(abs(current_potentiometer_value_Y - potentiometer_value_Y) >= MINIMUM_VARIATION){
      #if DEBUG_LEVEL > 1
      Serial.println(current_potentiometer_value_Y);
      #endif
      potentiometer_value_Y = current_potentiometer_value_Y;
    }
    if(abs(current_potentiometer_value_X - potentiometer_value_X) >= MINIMUM_VARIATION){
      #if DEBUG_LEVEL > 1
      Serial.println(current_potentiometer_value_X);
      #endif
      potentiometer_value_X = current_potentiometer_value_X;
    }
}

void Joystick::update_charthesian_coordinates() {
    sample_potentiometer_values();
    normalize_carthesian_coordinates();
    apply_zero_filter();
    //At the end, in actual_coordinates there will be the filtered coordinates
}

Joystick::Joystick(gpio_num_t Y_pin,gpio_num_t X_pin,unsigned int minimum_variation, float zero_threshold){

    MINIMUM_VARIATION = minimum_variation;
    ZERO_THRESHOLD = zero_threshold;
    joystick_Y_pin = Y_pin;
    joystick_X_pin = X_pin;

    /*
    gpio_reset_pin(joystick_Y_pin);
    gpio_reset_pin(joystick_X_pin);

    gpio_set_direction(joystick_Y_pin, GPIO_MODE_INPUT);
    gpio_set_direction(joystick_X_pin, GPIO_MODE_INPUT);
    */

    /*CHANGE INTO analogRead() and pinMode()*/
    pinMode(joystick_Y_pin,INPUT_PULLUP);
    pinMode(joystick_X_pin,INPUT_PULLUP);

    potentiometer_value_Y = potentiometer_value_X = -1; //To recognise first reading

}

float Joystick::get_X_value(){
    return actual_coordinates.x;
}

float Joystick::get_Y_value(){
    return actual_coordinates.y;
}

int Joystick::get_X_potentiometer_value(){
    return potentiometer_value_X;
}

int Joystick::get_Y_potentiometer_value(){
    return potentiometer_value_Y;
}

//Initial calibration and then auto-calibration when reading is > thresholds
void Joystick::calibrate_joystick(){
  Serial.printf("Start calibration\n");
  /*Serial.printf("RIGHT\n");
  buzz();
  start_millis = millis();
  while( (millis() - start_millis) < 1000 * 10){
    x = analogRead(joystick_X_pin);

    if(x > ABSOLUTE_MAX_X){
      ABSOLUTE_MAX_X = x;
      Serial.printf("MAX_X %u\n",ABSOLUTE_MAX_X);
    }
  }
  Serial.printf("LEFT\n");
  buzz();
  start_millis = millis();
  while( (millis() - start_millis) < 1000 * 10){
    x = analogRead(joystick_X_pin);
    if(x < ABSOLUTE_MIN_X){
      ABSOLUTE_MIN_X = x;
      Serial.printf("MIN _X %u\n",ABSOLUTE_MIN_X);
    }
  }
  Serial.printf("UP\n");
  buzz();
  start_millis = millis();
  while( (millis() - start_millis) < 1000 * 10){
    y = analogRead(joystick_Y_pin);

    if(y > ABSOLUTE_MAX_Y){
      ABSOLUTE_MAX_Y = y;
      Serial.printf("MAX_Y %u\n",ABSOLUTE_MAX_Y);
    }
  }
  Serial.printf("DOWN\n");
  buzz();
  start_millis = millis();
  while( (millis() - start_millis) < 1000 * 10){
    y = analogRead(joystick_Y_pin);
    if(y < ABSOLUTE_MIN_Y){
      ABSOLUTE_MIN_Y = y;
      Serial.printf("MIN_Y %u\n",ABSOLUTE_MIN_Y);
    }
  }*/
  //buzz();
    unsigned long start_millis = millis();
    uint16_t x,y, ABS_MAX_X = 0, ABS_MAX_Y = 0, ABS_MIN_X = UINT16_MAX, ABS_MIN_Y = UINT16_MAX;

    while( (millis() - start_millis) < 1000 * 30){
        x = analogRead(joystick_X_pin);
        y = analogRead(joystick_Y_pin);

        if(y > ABS_MAX_Y){
            ABS_MAX_Y = y;
            Serial.printf("MAX_Y %u\n",ABS_MAX_Y);
        }
        if(y < ABS_MIN_Y){
            ABS_MIN_Y = y;
            Serial.printf("MIN_Y %u\n",ABS_MIN_Y);
        }

        if(x > ABS_MAX_X){
            ABS_MAX_X = x;
            Serial.printf("MAX_X %u\n",ABS_MAX_X);
        }
        if(x < ABS_MIN_X){
            ABS_MIN_X = x;
            Serial.printf("MIN _X %u\n",ABS_MIN_X);
        }
    }

    Serial.printf("ABS_MAX_X %u\nABS_MIN_X %u\nABS_MAX_Y %u\nABS_MIN_Y %u\n",ABS_MAX_X,ABS_MIN_X,ABS_MAX_Y,ABS_MIN_Y);

}