/*
TunnelSensor.h
Interface to light sensor
*/
#include "util.h"

class TunnelSensor{
    public:
        void sensorSetup();//sets up input pins
        bool TunnelDetected();//returns true if in tunnel
        bool WallCollisionLeft();//returns true if collided with the left wall
        bool WallCollisionRight();//returns true if collided with the right wall
    private:
        int TunnelSenseBuffer=0;//internal counter causing delay when leaving tunnel
};