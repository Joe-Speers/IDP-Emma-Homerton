#include "ultra_sound.ino"


void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication
}
void loop() {
   while (distance >= 20) {//normal motion
   }
   while (distance < 20) {//recovery mode
   }

   //block detection in sensing block



   


}