#include <Wire.h>
#include "include/TunnelSensor.h"
#include "include/util.h"

void TunnelSensor::sensorSetup(){
    pinMode(LIGHT_SENSOR,INPUT);
}
bool TunnelSensor::TunnelDetected(){
    if (digitalRead(LIGHT_SENSOR) == HIGH)
    {
        return true;
    }
    if (digitalRead(LIGHT_SENSOR) == LOW)
    {
        return false;
    }
}