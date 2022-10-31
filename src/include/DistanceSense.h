/*

*/
#pragma once
#include "util.h"

class DistanceSense{
    public:
        void SensorSetup();     //Sets up sensors
        float ReadUltrasoundDistance();   //Reads distance from Ultrasound sensor in cm
        float ReadIRDistance();  //reads distance from Infrared sensor
    private:
        float val;
        float IR_distance;
        float average;
        float distanceVal;
        float previous_readings[IR_AVERAGE_READINGS];
};