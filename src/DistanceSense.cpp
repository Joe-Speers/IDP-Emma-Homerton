/*
DistanceSense.cpp
See header file for description
*/
#include "include/DistanceSense.h"
#include "include/util.h"
#include <Wire.h>

//Setup pin inputs / outputs
void DistanceSense::SensorSetup() {
  pinMode(ULTRASOUND_TRIGGER_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(ULTRASOUND_ECHO_PIN, INPUT); // Sets the echoPin as an Input
  pinMode(IR_SENSOR_PIN, INPUT);// sets the IR sensor as an input
}

//Read Ultrasound distance in cm, this call takes ULTRASOUND_TIMEOUT or less time to complete.
//If this times out, then INVALID_READING is returned
float DistanceSense::ReadUltrasoundDistance() {
  // Clears the trigPin
  digitalWrite(ULTRASOUND_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 50 micro seconds
  digitalWrite(ULTRASOUND_TRIGGER_PIN, HIGH);
  delayMicroseconds(50);
  digitalWrite(ULTRASOUND_TRIGGER_PIN, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(ULTRASOUND_ECHO_PIN, HIGH,ULTRASOUND_TIMEOUT);
  // Calculating the distance
  float distance = duration * 0.034 / 2;
  if(distance ==0){
    return INVALID_READING;
  }
  return distance;
}

//Read Infrared distance in cm. This has no significant delay unlike ultrasound.
//If the distance is too far then INVALID_READING is returned
//If distance is within 20cm then the reading is invalid and unstable.
float DistanceSense::ReadIRDistance() {
  float IRReading = analogRead(IR_SENSOR_PIN);
  //convert reading to a distance in cm
  float distanceVal = (11805/(IRReading - 38.013))-2;
  if (distanceVal > 90 ){
    distanceVal = (12334/(IRReading - 26.763))-2.5;
  }
  //find the average reading over IR_AVERAGE_READINGS to remove noise
  float average=distanceVal;
  for (int i = IR_AVERAGE_READINGS -1; i > 0; i--){
      previous_readings[i] = previous_readings[i - 1];
      average+=previous_readings[i];
  }
  previous_readings[0] = distanceVal;
  average /= IR_AVERAGE_READINGS;

  float IR_distance=average;
  //check reading is within valid range
  if (IR_distance<=0 || IR_distance> 160) { 
    return INVALID_READING;
  }
  else {
    return IR_distance;
  }
}