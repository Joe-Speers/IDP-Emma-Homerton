/*
LineSensor.cpp
See header file for description
*/

#pragma once

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "include/util.h"
#include "include/LineSensor.h"


//Setup input pins for line sensors
void LineSensor::LineSensorSetup() {
  pinMode(LINE_SENSOR_PIN,INPUT); //connectes to subtraction circuit, used for PID control (analog)
  pinMode(JUNCTION_SENSOR_PIN, INPUT); //digital input connecting through a schmitt trigger to the junction sensor
}

//Resets PID Integral and derivitive to zero, used when re-discovering line
void LineSensor::ResetPID(){
  integral=0;
  for(int i=0;i<DERIVITIVE_PREVIOUS_READINGS_TO_AVERAGE;i++){
    error_array[i]=0;
  }
}

//Peforms PID calculation based upon readings from line sensors. Returns a correction value between -1 and 1
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

//Returns if a line is detected based upon the noise present in the reading
bool LineSensor::isLineDetected(){
  //if the derivitive component is high then it is likely the robot is on the line
  //if the integral is close to zero then the robot is likely on the line
  //if the proporional value is above a baseline then the robot is on the line
  if(derivative>DERIVITIVE_LINE_SENSE_THRESHOLD || derivative< -DERIVITIVE_LINE_SENSE_THRESHOLD ||(integral<=INTEGRAL_LINE_SENSE_REGION && integral>=-INTEGRAL_LINE_SENSE_REGION && false) || (error>PROPORTIONAL_LINE_SENSE_REGION || error< -PROPORTIONAL_LINE_SENSE_REGION)){
    LostLineCounter=0;
    return true;
  } else{
    //use a counter system to remove noise.
    LostLineCounter+=1;
    if(LostLineCounter>LINE_LOST_COUNT){
      return false;
    } else{
      return true;
    }
  }
}

//Returns true if a junction is detected
bool LineSensor::juntionDetect() {
  if (digitalRead(JUNCTION_SENSOR_PIN) == LOW)
  {
    return true;
  }
  if (digitalRead(JUNCTION_SENSOR_PIN) == HIGH)
  {
    return false;
    
  }
}
