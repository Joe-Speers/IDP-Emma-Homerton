#include <Wire.h>
#include "include/TunnelSensor.h"
#include "include/util.h"

void TunnelSensor::sensorSetup(){
    pinMode(LIGHT_SENSOR,INPUT);
    pinMode(LEFT_BUMPER_PIN,INPUT);
    pinMode(RIGHT_BUMPER_PIN,INPUT);
}
int sticky=0;
bool TunnelSensor::TunnelDetected(){
    if (digitalRead(LIGHT_SENSOR) == HIGH)
    {
        sticky=0;
        return true;
    }
    if (digitalRead(LIGHT_SENSOR) == LOW)
    {
        sticky+=1;
        if(sticky>40){
            return false;
        }
        return true;;
    }
}
bool TunnelSensor::WallCollisionLeft(){
    if(digitalRead(LEFT_BUMPER_PIN) == HIGH){
        return true;
    }
    if(digitalRead(LEFT_BUMPER_PIN) == LOW){
        return false;
    }
}
bool TunnelSensor::WallCollisionRight(){
    if(digitalRead(RIGHT_BUMPER_PIN) == HIGH){
        return true;
    }
    if (digitalRead(RIGHT_BUMPER_PIN) == LOW){
        return false;
    }
}