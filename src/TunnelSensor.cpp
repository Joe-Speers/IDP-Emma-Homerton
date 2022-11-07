/*
TunnelSensor.cpp
See header file for description
*/
#include <Wire.h>
#include "include/TunnelSensor.h"
#include "include/util.h"

//sets up input pins
void TunnelSensor::sensorSetup(){
    pinMode(LIGHT_SENSOR,INPUT);
    pinMode(LEFT_BUMPER_PIN,INPUT);
    pinMode(RIGHT_BUMPER_PIN,INPUT);
}

//returns true if tunnel is detected
bool TunnelSensor::TunnelDetected(){
    if (digitalRead(LIGHT_SENSOR) == HIGH)
    {
        TunnelSenseBuffer=0;
        return true;
    }
    if (digitalRead(LIGHT_SENSOR) == LOW)
    {
        TunnelSenseBuffer+=1;
        if(TunnelSenseBuffer>40){//only lose tunnel if light sensor is low for 400ms
            return false;
        }
        return true;
    }
}

//returns true if collided with the left wall
bool TunnelSensor::WallCollisionLeft(){
    if(digitalRead(LEFT_BUMPER_PIN) == HIGH){
        return true;
    }
    if(digitalRead(LEFT_BUMPER_PIN) == LOW){
        return false;
    }
}
//returns true if collided with the right wall
bool TunnelSensor::WallCollisionRight(){
    if(digitalRead(RIGHT_BUMPER_PIN) == HIGH){
        return true;
    }
    if (digitalRead(RIGHT_BUMPER_PIN) == LOW){
        return false;
    }
}