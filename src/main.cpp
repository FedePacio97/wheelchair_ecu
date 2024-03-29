#include <Arduino.h>
#include "ECU.h"
#include "InterfaceEngineCU.h"
#include "IMU.h"
#include "WiFi.h" //For BLE communication
#include <esp_now.h>
#include "messages.h"
/*
  DEBUG_LEVEL
    2 high
    1 medium
    0 none

*/

//#define DEBUG_LEVEL 2

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

//#define PLOTTING 1
//#define PLOTTING 

#ifdef PLOTTING
//For plotting
#include "Plotter.h"
Plotter p; // create plotter
float ACC_X, ACC_Y;
#endif

//define ECU object
ECU ecu;
// define InterfaceEngineCU object
InterfaceEngineCU interfaceEngineCU;
// // define IMU object
IMU imu;

// define two tasks for Blink & AnalogRead
void TaskBlink( void *pvParameters );
void TaskHandleWheelchairMovement( void *pvParameters );
void TaskIMUupdates( void *pvParameters);
void TaskCheckAndHandleMotorsParameters( void *pvParameters);
void TaskSendStabilityInfoForScreen (void *pvParameters);

//Define task handle (used to be the target of xNotify(.))
TaskHandle_t xHandlerOfTaskHandleWheelchairMovement = NULL;
TaskHandle_t xHandlerOfTaskCheckConsistencyAmongRPM_IMUvelocity = NULL;

//Due to absence of xTaskNotifyIndexed in freertos for esp32, implement it with 2 distinct variables
//protected by 2 distinct mutexes
SemaphoreHandle_t xSemaphore_mutex_ECU_EngineCU_notification, xSemaphore_mutex_ECU_IMU_notification;
uint32_t ECU_EngineCU_notification,ECU_IMU_notification;

//Used to access ahsr_IMU_updates
SemaphoreHandle_t xSemaphore_mutex_send_IMU_updates;
AHSR ahsr_IMU_updates = {0};
ACC_GYRO_MAG acc_gyro_mag_IMU_updates = {0};

/* Queue used to receive RPM messages from Joystick. */
QueueHandle_t xRPM_From_Joystick_Queue = NULL;

//For debugging purpose
#define VESC_RX_LED 26
#define VESC_LX_LED 25
#define REMOTE_CONTROLLER_LED 32



// the setup function runs once when you press reset or power the board
void setup() {
  
  // initialize serial communication at 115200 bits per second:
  #ifdef PLOTTING
  p.Begin(); // start plotter
  p.AddTimeGraph( "ACC", 4500, "acc_x", ACC_X, "acc_y", ACC_Y ); // add any graphs you want
  #else
  Serial.begin(115200);
  #endif

  #if DEBUG_LEVEL > 1
  Serial.println("Gnamo!");
  #endif


  interfaceEngineCU.initialize_serial_ports_for_UART_communication_with_VESCs();

  //Initialize buzzer queue
  // Create the queue used to send complete struct BuzzerMessage structures.
  xRPM_From_Joystick_Queue = xQueueCreate(
                         // The number of items the queue can hold. 
                         10,
                         // Size of each item is big enough to hold the
                         //whole structure.
                         sizeof(RPM_message) );

  interfaceEngineCU.initialize_bluetooth_communication_with_joystick();
                     

  xSemaphore_mutex_ECU_EngineCU_notification = xSemaphoreCreateMutex();
  xSemaphore_mutex_ECU_IMU_notification = xSemaphoreCreateMutex();
  xSemaphore_mutex_send_IMU_updates = xSemaphoreCreateMutex();
  
  if( xSemaphore_mutex_ECU_EngineCU_notification == NULL )
  {
      // The semaphore wasn't created successfully
      //buzz();
      //abort
  }
  if( xSemaphore_mutex_ECU_IMU_notification == NULL )
  {
      // The semaphore wasn't created successfully
      //buzz();
      //abort
  }
  if( xSemaphore_mutex_send_IMU_updates == NULL )
  {
      // The semaphore wasn't created successfully
      //buzz();
      //abort
  }

  pinMode(VESC_RX_LED, OUTPUT);
  pinMode(VESC_LX_LED, OUTPUT);
  pinMode(REMOTE_CONTROLLER_LED, OUTPUT);

  // Now set up two tasks to run independently.
  // xTaskCreatePinnedToCore(
  //   TaskBlink
  //   ,  "TaskBlink"   // A name just for humans
  //   ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
  //   ,  NULL
  //   ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  //   ,  NULL 
  //   ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskIMUupdates
    ,  "IMUupdates"
    ,  2048  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);
  
  xTaskCreatePinnedToCore(
    TaskHandleWheelchairMovement
    ,  "HandleWheelchairMovement"
    ,  2048  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  &xHandlerOfTaskHandleWheelchairMovement 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskCheckAndHandleMotorsParameters
    ,  "CheckAndHandleMotorsParameters"
    ,  2048  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskSendStabilityInfoForScreen
    ,  "SendStabilityInfoForScreen"
    ,  2048  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL
    ,  ARDUINO_RUNNING_CORE);

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
  #ifdef PLOTTING
  p.Plot();
  #endif
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBlink(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
    
  If you want to know what pin the on-board LED is connected to on your ESP32 model, check
  the Technical Specs of your board.
*/

  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay(1000 /portTICK_PERIOD_MS);  // 1000ms delay
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay(1000 /portTICK_PERIOD_MS);  // 1000ms delay
  }
}

void TaskHandleWheelchairMovement(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
/*
  TaskHandleWheelchairMovement
  Check if IMU has signalled a stability issue
  If so -> execute safety procedure
  If not,
    read RPM values from Queue filled by BLE handler
    send RPM values to VESCs via UART
*/

  RPM_message rpm_message;
  uint32_t current_ECU_IMU_notification = 0;
  for (;;)
  {

    //Extract ECU_IMU_notification
    xSemaphoreTake( xSemaphore_mutex_ECU_IMU_notification, portMAX_DELAY );
    current_ECU_IMU_notification = ECU_IMU_notification;
    //reset the "notification"
    ECU_IMU_notification = 0;
    xSemaphoreGive( xSemaphore_mutex_ECU_IMU_notification );

    if(current_ECU_IMU_notification == 0){
      //No pending notification -timed out before a notification was received
      //Behave in the normal way
      #if DEBUG_LEVEL > 2
      Serial.printf("[TaskHandleWheelchairMovement]: No notifications\n");
      #endif

      if( xRPM_From_Joystick_Queue != NULL )
      {
        /* Receive a message from the created queue to hold complex struct RPM_message
        structure.  Do not block otherwise notifications may be missed.
        The value is read into a struct RPM_message variable, so after calling
        xQueueReceive() rpm_message will hold a copy of xMessage, in case a message has been found in the queue. */
        if(
          xQueueReceive( xRPM_From_Joystick_Queue, 
                         &( rpm_message ),
                         0) == pdPASS)
        {
          #if DEBUG_LEVEL > 1
          Serial.printf("[RECEIVED] RPM_lx %d \t RPM_rx %d\n",rpm_message.RPM_LX, rpm_message.RPM_RX);
          #endif

          // if( (rpm_message.RPM_LX == 0) && (rpm_message.RPM_RX==0) ){
          //   //The user wants to stop
          //   //should check if real RPM are 0. In case they go negative (check how to get it), set brake
          // }else{
          //   //classic behaviour
          //   interfaceEngineCU.set_RPM_motors(rpm_message.RPM_LX,rpm_message.RPM_RX);
          // }

          interfaceEngineCU.set_RPM_motors(rpm_message.RPM_LX,rpm_message.RPM_RX);
          
        }
      }
    
    }else{

      //Side effect of branch else:
      //If notifications are detected, the ECU doesn't send commands to motors.
      //Positive since dangerous situation
      #if DEBUG_LEVEL > 1
      Serial.printf("[TaskHandleWheelchairMovement]: NOTIFICATION!\n");
      #endif

      if(current_ECU_IMU_notification == WHEELCHAIR_RISK_OVERTURNING_PITCH){ //Sent by the IMU task
        //The wheelchair is about to overturn
        #if DEBUG_LEVEL > 1
        Serial.printf("[TaskHandleWheelchairMovement]: Received WHEELCHAIR_RISK_OVERTURNING_PITCH\n");
        #endif
          //DO SOMETHING!
          //TRY TO SPIN THE MOTOR IN OPPOSITE DIRECTION
          //ALERT ON SCREEN TO PUSH RED BUTTON
          //ENABLE ALARM BUZZER
        
      }else if(current_ECU_IMU_notification == WHEELCHAIR_RISK_OVERTURNING_ROLL){ //Sent by the IMU task
        //The wheelchair is about to overturn
        #if DEBUG_LEVEL > 1
        Serial.printf("[TaskHandleWheelchairMovement]: Received WHEELCHAIR_RISK_OVERTURNING_ROLL\n");
        #endif

      }else if(current_ECU_IMU_notification == WHEELCHAIR_OVERTURNED_PITCH){ //Sent by the IMU task
        //The wheelchair is overturned
        #if DEBUG_LEVEL > 1
        Serial.printf("[TaskHandleWheelchairMovement]: Received WHEELCHAIR_OVERTURNED_PITCH\n");
        #endif
  
      }else if(current_ECU_IMU_notification == WHEELCHAIR_OVERTURNED_ROLL){ //Sent by the IMU task
        //The wheelchair is overturned
        #if DEBUG_LEVEL > 1
        Serial.printf("[TaskHandleWheelchairMovement]: Received WHEELCHAIR_OVERTURNED_ROLL\n");
        #endif

      }
    }

    vTaskDelay(15/portTICK_PERIOD_MS);  // 50ms delay == 1/(10 * 10^-3) = 100Hz
  }
}

void TaskCheckAndHandleMotorsParameters(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

/*
  TaskCheckAndHandleMotorsParameters
  Retrieve motor value from VESCs and aggregate
  Handle them by sending to ESP32 in charge of handling screen and joystick.
*/

  Telemetry_info_from_VESC VESC_LX_info, VESC_RX_info;
  for (;;) // A Task shall never return or exit.
  {
    
    bool res = interfaceEngineCU.retrieve_motors_info(VESC_LX_info, VESC_RX_info);
    if(res){

      Telemetry_message_sent_on_BLE telemetry_sent_on_BLE;
      telemetry_sent_on_BLE.OPCODE = TELEMETRY_OPCODE;
      //Calculate costant for km
      telemetry_sent_on_BLE.telemetry_message.km = ( (VESC_LX_info.tachometerAbs + VESC_RX_info.tachometerAbs)/2 ) / TACHOMETER_MOTOR_PER_WHEEL_ROUND * (2 * PI * R) / 1000; ///1000 since km
      //rpm = Erpm * pole_pairs
      //Vehicle speed [km/h]= 0.1885 * Wheel RPM * diameter of the tire
      //RPM_motor * Radius_motor [m] = RPM_wheel * Radius_wheel [m]
      //find wheel RPM from RPM of motor
      telemetry_sent_on_BLE.telemetry_message.speed = ( (VESC_LX_info.rpm / 14 * 0.062 / 0.6) + (VESC_RX_info.rpm / 14 * 0.062 / 0.6) ) / 2;

      //Battery level = current_voltage * 100 / max_voltage_battery
      //Fully charged battery = 29.2 V
      telemetry_sent_on_BLE.telemetry_message.battery_level = ( (VESC_LX_info.inpVoltage + VESC_RX_info.inpVoltage) / 2 ) * 100 / 29.2;
      
      //Total Ah = 27.2 Ah
      //Send data via BLE
      esp_now_send(joystick_controller_MAC, (uint8_t *) &telemetry_sent_on_BLE, sizeof(Telemetry_message_sent_on_BLE));

      #if DEBUG_LEVEL > 2
      Serial.printf("Battery level LX%f\n",VESC_LX_info.inpVoltage);
      Serial.printf("Battery level RX%f\n",VESC_RX_info.inpVoltage);

      Serial.printf("Tachometer LX%f\n",VESC_LX_info.tachometer);
      Serial.printf("Tachometer RX%f\n",VESC_RX_info.tachometer);

      Serial.printf("Tachometer ABS LX%f\n",VESC_LX_info.tachometerAbs);
      Serial.printf("Tachometer ABS RX%f\n",VESC_RX_info.tachometerAbs);

      Serial.printf("rpm LX%f\n",VESC_LX_info.rpm);
      Serial.printf("rpm RX%f\n",VESC_RX_info.rpm);
      #endif

    }

    vTaskDelay(500 /portTICK_PERIOD_MS);  // 1000ms delay
  }
}


void TaskIMUupdates(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
  /*
  TaskIMUupdates
  Retrieve values from IMU
  If stability issue is detected -> signal to TaskHandleWheelchairMovement
  Make available latest IMU values
  */
  
  
  //TaskIMUupdates
  //Handles IMU updates and check if pitch and roll are below safe thresholds: if not so, send notification to ECU


  TickType_t xLastWakeTime;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    // Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, 20/portTICK_PERIOD_MS ); //Fixed time execution at KEEP_ALIVE_PERIOD_ms (1000ms by default) rate
    //May execute at rate of 1/50ms, i.e 20Hz

    // Perform action here
    imu.updateSensors();
    uint16_t stability_result = imu.check_stability();
    if(stability_result == WHEELCHAIR_STABLE){
      //Correct behaviour
      #if DEBUG_LEVEL > 1
      Serial.printf("[TaskIMUupdates] Wheelchair is stable!\n");
      #endif
    }else{
      //Uncorrect behaviour
      //Send a "notification" to the TaskHandleWheelchairMovement
      //Extract ECU_IMU_notification
      xSemaphoreTake( xSemaphore_mutex_ECU_IMU_notification, portMAX_DELAY );
      ECU_IMU_notification = stability_result;
      xSemaphoreGive( xSemaphore_mutex_ECU_IMU_notification );
      
      #if DEBUG_LEVEL > 1
      Serial.printf("[TaskIMUupdates] Wheelchair is NOT STABLE!\n");
      #endif


    }
    
    //Send in queue pitch, roll (and yaw, not currently used by screen) for TaskSendStabilityInfoForScreen
    //In truth, we use only a shared variable since TaskIMUupdates runs as faster as possible so it will fulfill the queue very soon.
    //Moreover, it is a waste o CPU time to make TaskSendStabilityInfoForScreen run as fast as TaskIMUupdates since we are not interested in updating
    //the screen so fast.

    //In this way, TaskSendStabilityInfoForScreen will extract the latest available info
    xSemaphoreTake( xSemaphore_mutex_send_IMU_updates, portMAX_DELAY );
    ahsr_IMU_updates = imu.get_ahsr();
    acc_gyro_mag_IMU_updates = imu.get_acc_gyro_mag();
    xSemaphoreGive( xSemaphore_mutex_send_IMU_updates );


  }
}

void TaskSendStabilityInfoForScreen(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
/*
  TaskSendStabilityInfoForScreen
  Check xIMU_Info_Queue to extract available mex to be sent toward the screen

*/

  AHSR ahsr_received_from_IMU;
  ACC_GYRO_MAG acc_gyro_mag_received_from_IMU;
  for (;;)
  {

    xSemaphoreTake( xSemaphore_mutex_send_IMU_updates, portMAX_DELAY );
    ahsr_received_from_IMU = ahsr_IMU_updates;
    acc_gyro_mag_received_from_IMU = acc_gyro_mag_IMU_updates;
    xSemaphoreGive( xSemaphore_mutex_send_IMU_updates );

    //send info to wheelchair_joystick_screen
    Stability_message_sent_on_BLE stability_info_sent_on_BLE;
    stability_info_sent_on_BLE.OPCODE = STABILITY_INFO_OPCODE;
    stability_info_sent_on_BLE.stability_info.pitch = ahsr_received_from_IMU.pitch;
    stability_info_sent_on_BLE.stability_info.roll = ahsr_received_from_IMU.roll;

    stability_info_sent_on_BLE.stability_info.accel_x = acc_gyro_mag_received_from_IMU.accel_x;
    stability_info_sent_on_BLE.stability_info.accel_y = acc_gyro_mag_received_from_IMU.accel_y;
    stability_info_sent_on_BLE.stability_info.accel_z = acc_gyro_mag_received_from_IMU.accel_z;
    stability_info_sent_on_BLE.stability_info.gyro_x = acc_gyro_mag_received_from_IMU.gyro_x;
    stability_info_sent_on_BLE.stability_info.gyro_y = acc_gyro_mag_received_from_IMU.gyro_y;
    stability_info_sent_on_BLE.stability_info.gyro_z = acc_gyro_mag_received_from_IMU.gyro_z;
    stability_info_sent_on_BLE.stability_info.mag_x = acc_gyro_mag_received_from_IMU.mag_x;
    stability_info_sent_on_BLE.stability_info.mag_y = acc_gyro_mag_received_from_IMU.mag_y;
    stability_info_sent_on_BLE.stability_info.mag_z = acc_gyro_mag_received_from_IMU.mag_z;

    #ifdef PLOTTING
    ACC_X = stability_info_sent_on_BLE.stability_info.accel_x;
    ACC_Y = stability_info_sent_on_BLE.stability_info.accel_y;
    #endif

    #if DEBUG_LEVEL > 2
    Serial.printf("pitch %d \t roll %d\naccX %f\taccY %f\taccZ %f\ngyroX %f\tgyroY %f\tgyroZ %f\nmagX %f\tmagY %f\tmagZ %f\n",
    stability_info_sent_on_BLE.stability_info.pitch, stability_info_sent_on_BLE.stability_info.roll,
    stability_info_sent_on_BLE.stability_info.accel_x, stability_info_sent_on_BLE.stability_info.accel_y, stability_info_sent_on_BLE.stability_info.accel_z,
    stability_info_sent_on_BLE.stability_info.gyro_x, stability_info_sent_on_BLE.stability_info.gyro_y, stability_info_sent_on_BLE.stability_info.gyro_z,
    stability_info_sent_on_BLE.stability_info.mag_x, stability_info_sent_on_BLE.stability_info.mag_y, stability_info_sent_on_BLE.stability_info.mag_z);
    #endif
    //send data via BLE
    esp_now_send(joystick_controller_MAC, (uint8_t *) &stability_info_sent_on_BLE, sizeof(Stability_message_sent_on_BLE));

    vTaskDelay(500/portTICK_PERIOD_MS);  // 50ms delay == 1/(50 * 10^-3) = 20Hz
  }
}




//TO DO
/*
SETUP PER JOYSTICK E AUTOCALIBRATION DURANTE USO SE VEDE CURRENT <> THRESHOLD ..missing
FREQUENCY JOYSTICK 20 HZ -> v
APPLY FILTER ON JOYSTICK -> v
*/