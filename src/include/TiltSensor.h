/*
TiltSensor.h
Contans an interface to the units IMU, returning the current orientation state
*/
#include "util.h"

class TiltSensor{
    public:

        enum TiltState{ //all possible orientations.
            HORIZONTAL = 0,
            TILT_UP = 1, //for going up ramp
            TILT_DOWN = 2, //for going down ramp
        };
    float y_average;// average y reading from tilt sensor

    void sensorSetup();//setup IMU
    void reset(); //reset state to horizontal
    TiltState getTilt(int dt_ms=0); //returns the current tilt state. Should be called continuosly

    private:
        TiltState lastState = HORIZONTAL; //stores the current orientation
        float previous_y_readings[TILT_AVERAGE_READINGS];//stores previois readings to generate average
        int buffer = 0; //a counter variable to ensure a minimum time delay between changing states
};