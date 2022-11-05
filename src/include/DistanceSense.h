/*
DistanceSense.h
Handles sensor input from ultrasound and Infrared distance sensors.
Converts readings into cm, or returns INVALID_READING if out of range.

Ultrasound sensor: HC-SR04
Infrared sensor: GP2Y0A02YK0F (150cm)
*/
#pragma once
#include "util.h"

class DistanceSense{
    public:
        void SensorSetup();     //Sets up sensors
        float ReadUltrasoundDistance();   //Reads distance from Ultrasound sensor in cm (delay of ULTRASOUND_TIMEOUT)
        float ReadIRDistance();  //reads distance from Infrared sensor in cm (no significant delay)
    private:
        float previous_readings[IR_AVERAGE_READINGS]; //stores previous readings so an average can be calculated
};