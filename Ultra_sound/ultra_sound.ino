#include <Filters.h> //to apply exponential recursive filter

// defines pins numbers
const int trigPin = 9;
const int echoPin = 10;
// defines variables
long duration;
int distance;
//Filter for float numbers, (a,b) a is weighting of filter (to be calibrated), b is initial value of filter
ExponentialFilter<float> FilteredDistance(90, 0);

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication
}
void loop() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor in cm
  Serial.print("Distance: ");
  Serial.println(distance);
  //applying filter
  FilteredDistance.Filter(distance);
  float SmoothDistance = FilteredDistance.Current();
  // print SmoothDistance in Serial monitor
  Serial.print("Smoothed Distance: ")
  Serial.println(SmoothDistance)
}