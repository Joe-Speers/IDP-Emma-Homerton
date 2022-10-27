/*

*/
#include "util.h"

class DistanceSense{
    public:
        void SensorSetup();     //Sets up Ultrasound sensor
        float ReadUltrasoundDistance();   //Reads distance from Ultrasound sensor in cm
        void IRSetup();  //sets up IR sensor
        float ReadIRDistance();  //reads distance from Infrared sensor
    private:
        float val;
        float IR_distance;
        float average;
        float distanceVal;
        float previous_readings[IR_AVERAGE_READINGS];
};