#include <Filters.h> //to apply exponential recursive filter
#include "include/DistanceSense.h"
#include "include/util.h"

//Filter for float numbers, (a,b) a is weighting of filter (to be calibrated), b is initial value of filter
//ExponentialFilter<float> FilteredDistance(90, 0);

void DistanceSense::SensorSetup() {
  pinMode(ULTRASOUND_TRIGGER_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(ULTRASOUND_ECHO_PIN, INPUT); // Sets the echoPin as an Input
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