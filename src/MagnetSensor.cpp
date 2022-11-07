/*
MagnetSensor.cpp
See header file for description
*/

#include <Wire.h>
#include "include/MagnetSensor.h"
#include "include/util.h"


//Sets up magnet sensor input pin
void MagnetSensor::sensorSetup(){
    pinMode(MAGNET_SENSOR_PIN,INPUT);
}

//Called continuously, returns true if a magnet has been detected in the last 5 seconds
bool MagnetSensor::MagnetDetected(){
    if (digitalRead(MAGNET_SENSOR_PIN) == LOW)
    {
        detected_magnet_countdown-=1;
    }
    if (digitalRead(MAGNET_SENSOR_PIN) == HIGH)
    {
        detected_magnet_countdown=500;
    }
    if(detected_magnet_countdown<0)detected_magnet_countdown=0;
    if(detected_magnet_countdown>0){
        return true;
    } else {
        return false;
    }
}
