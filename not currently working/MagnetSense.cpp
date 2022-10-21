#include "include/MagnetSense.h"
#include "include/util.h"

void Magnetsense::MagnetSensorSetup(){
  Serial.begin(9600);
  pinMode(Magnet_Red_LED,OUTPUT);//control red led turn on when magmet is found
  pinMode(NonMagnet_Green_LED,OUTPUT);//control red led turn on when magmet is found
  pinMode(HALL_SENSOR_PIN,INPUT);
}
int void Magnetsense::MagnetSensorWork(){
  hallreading=analogRead(HALL_SENSOR_PIN);
  delay(500);//to be discussed the interval between 2 detecting
  if(hallreading<510){//can adjust accuracy
    digitalWrite(Magnet_Red_LED,HIGH);
    result=1;// Magnet is found, red led is on, result is set to be 1.
  }
  else{
    digitalWrite(NonMagnet_Green_LED,HIGH);
    result=0;// Magnet is not found, green led is on, result is set to be 0.
  }
  }