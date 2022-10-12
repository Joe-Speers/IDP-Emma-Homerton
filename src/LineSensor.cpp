/*
This program uses PID control to follow a line.
See Linear Systems and Control handout 6, page 14 onwards for PID control info. Effectivly uses the derivitive and integral of the error (in proportions derivitive_K and INTEGRAL_K)
as well as proportional control (PROPORTIONAL_K) to set the correcting value.
Ideally motor control should be removed from this file and implemented seperatly
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include "include/LineSensor.h" // see this file for publicly accessable variables

int LINE_SENSOR_PIN = A0;

//PID control constants
float PROPORTIONAL_K=1;
float INTEGRAL_K=7;
float derivitive_K=0.003;
float INTEGRAL_LIMIT=(1/INTEGRAL_K)/6;// hard limit on integral size

//Settings for line sensor pin
double LINE_SENSE_MIDDLE = 531; //what reading should be treated as the 'LINE_SENSE_MIDDLE' of the line
int LINE_SENSE_MAX_AMPLITUDE = 500; // aproximate max +- possible reading about LINE_SENSE_MIDDLE. Range is therefore 2*LINE_SENSE_MAX_AMPLITUDE
double ERROR_DEAD_SPOT=0.01; //fraction of reading (between 0 and 1) to discard. e.g. 0.01 means if reading within 1% of 0 then treat as 0.


//global variable
float error_array[5]; // stores the last 5 values of 'error', used to calculate derivitive


void LineSensor::LineSensorSetup() {
  
  pinMode(LINE_SENSOR_PIN,INPUT);
}

void LineSensor::LineSensorUpdate(int dt_micros) {
  double dt=(double)dt_micros/1000000;//calculate dt in seconds
  //take reading of line sensor and calculate error
  differential_reading = analogRead(LINE_SENSOR_PIN);
  error = (LINE_SENSE_MIDDLE-differential_reading)/LINE_SENSE_MAX_AMPLITUDE; // normalises reading to approximatly -1 to 1.
  //removes error if close to LINE_SENSE_MIDDLE
  if(error==0 or (error<=ERROR_DEAD_SPOT && error>=-ERROR_DEAD_SPOT)){
    error=0;
  }
  //add error to array of readings and shuffles the rest up one spot
  for (int i = 4; i > 0; i--){
    error_array[i] = error_array[i - 1];
  }
  error_array[0]=error;
  //// ### PID CONTROL ###
  //Derivitive: calculate slope of error based on the average slope of the last few readings.
  derivitive=0;
  for(int i=0;i<4;i++){
    derivitive+=error_array[i+1]-error_array[i];
  }
  derivitive=derivitive/(4*dt);
  //Integral: updates the integral of the error
  integral+=error*dt;
  if(integral>INTEGRAL_LIMIT) integral=INTEGRAL_LIMIT;
  if(integral<-INTEGRAL_LIMIT) integral=-INTEGRAL_LIMIT;
  //Proportional is just -k * error
  // Calculate correction value
  correction=(-PROPORTIONAL_K*error) + (-INTEGRAL_K*integral) +(-derivitive_K*derivitive);
  //Serial.println(correction);
  //chop off anything above 1 or below -1.
  if(correction>1){
    correction=1;
  }
  if(correction<-1){
    correction=-1;
  }
  
}
