/*
This program uses PID control to follow a line.
See Linear Systems and Control handout 6, page 14 onwards for PID control info. Effectivly uses the derivative and integral of the error (in proportions derivative_k and integral_k)
as well as proportional control (proportional_k) to set the correcting value.
Ideally motor control should be removed from this file and implemented seperatly
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include "include/util.h" //contains some key constants used by this file
#include "include/LineSensor.h" // see this file for publicly accessable variables


void LineSensor::LineSensorSetup() {
  pinMode(LINE_SENSOR_PIN,INPUT);
  pinMode(JUNCTION_SENSOR_PIN, INPUT);
}

void LineSensor::ResetPID(){
  integral=0;
  //also reset error values (todo)
}

double LineSensor::PIDLineFollowCorrection(int dt_micros) {
  double dt=(double)dt_micros/1000000;//calculate dt in seconds, this is the time elapsed since the last call

  // ### Sensor reading and normalisation ###

  //take reading of line sensor and normalise to be between -1 and 1
  differential_reading = analogRead(LINE_SENSOR_PIN);
  error = -(double)(LINE_SENSE_MIDDLE-differential_reading)/LINE_SENSE_MAX_AMPLITUDE; // normalises reading to approximatly -1 to 1.
  //creates a 'dead spot' around LINE_SENSE_MIDDLE
  if(error==0 or (error<=ERROR_DEAD_SPOT && error>=-ERROR_DEAD_SPOT)){
    error=0;
  }
  //add error to array of previous readings and shuffles the rest up one spot
  for (int i = DERIVITIVE_PREVIOUS_READINGS_TO_AVERAGE -1; i > 0; i--){
    error_array[i] = error_array[i - 1];
  }
  error_array[0]=error;

  //// ### PID CONTROL ###

  //derivative: calculate slope of error based on the average slope of the last few readings.
  derivative=0;
  for(int i=0;i<DERIVITIVE_PREVIOUS_READINGS_TO_AVERAGE-1;i++){
    derivative+=error_array[i+1]-error_array[i];
  }
  derivative=derivative/((DERIVITIVE_PREVIOUS_READINGS_TO_AVERAGE-1)*dt);
  //Integral: updates the integral of the error
  integral+=error*dt;
  if(integral>integral_limit) integral=integral_limit;
  if(integral<-integral_limit) integral=-integral_limit;
  //Proportional is just -proportional_k * error
  // Calculate correction value
  correction=(-proportional_k*error) + (-integral_k*integral) +(-derivative_k*derivative);
  //chop off anything above 1 or below -1.
  if(correction>1){
    correction=1;
  }
  if(correction<-1){
    correction=-1;
  }
  return correction;
}

bool LineSensor::juntionDetect() {
  
  if (digitalRead(JUNCTION_SENSOR_PIN == HIGH))
  {
    return 1;
  }
  if (digitalRead(JUNCTION_SENSOR_PIN == LOW)
  {
    return 0;
  }
}
