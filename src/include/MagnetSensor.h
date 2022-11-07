/*
MagnetSensor.h
Interface program for magnet sensor
*/
#include "util.h"

class MagnetSensor{
    public:
        void sensorSetup(); //Sets up magnet sensor input pin
        bool MagnetDetected();//Called continuously, returns true if a magnet has been detected in the last 5 seconds
    private:
        int detected_magnet_countdown=0; //counter for loosing magnet signal
};