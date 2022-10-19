#include "util.h"

class TiltSensor{
    public:
        enum TiltState{
            HORIZONTAL = 0,
            TILT_UP = 1,
            TILT_DOWN = 2,
        };
        
    void sensorSetup();
    TiltState getTilt(int dt); //returns the current tilt state
    private:
    TiltState lastState;
    float x, y, z;
    int buffer = 0;
}