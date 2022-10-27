#include <Filters.h> //to apply exponential recursive filter
#include "include/DistanceSense.h"
#include "include/util.h"

//Filter for float numbers, (a,b) a is weighting of filter (to be calibrated), b is initial value of filter
//ExponentialFilter<float> FilteredDistance(90, 0);

void DistanceSense::SensorSetup() {
  pinMode(ULTRASOUND_TRIGGER_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(ULTRASOUND_ECHO_PIN, INPUT); // Sets the echoPin as an Input
  pinMode(IR_SENSOR_PIN, INPUT);// sets the IR sensor as an input
}

void DistanceSense::IRSetup() {
//HERE pin is already in utils is this enough
}

float DistanceSense::ReadUltrasoundDistance() {
  // Clears the trigPin
  digitalWrite(ULTRASOUND_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(ULTRASOUND_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASOUND_TRIGGER_PIN, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(ULTRASOUND_ECHO_PIN, HIGH);
  // Calculating the distance
  float distance = duration * 0.034 / 2;
  //applying filter
  //FilteredDistance.Filter(distance);
  //float SmoothDistance = FilteredDistance.Current();
  // print SmoothDistance in Serial monitor
  return distance;
}

float DistanceSense::ReadIRDistance() {
  //HERE
  val = analogRead(IR_SENSOR_PIN);
  distanceVal = (11805/(val - 36.013));
  if (distanceVal >90 ){
    distanceVal = (12334/(val - 23.763));
  }

 
  average=distanceVal;
  for (int i = IR_AVERAGE_READINGS -1; i > 0; i--){
      previous_readings[i] = previous_readings[i - 1];
      average+=previous_readings[i];
  }
  previous_readings[0] = distanceVal;
  average /= IR_AVERAGE_READINGS;
  if (0 < IR_distance < 160) { //accounts for errors in non0linear region when outside of range to prevent spiking
    Serial.print("Out of range");
  }
  else {
    return IR_distance;
  } //returns distances that are in the range
  //nb errors incured when distance <20cm... need to reverse robot before scanning?
}