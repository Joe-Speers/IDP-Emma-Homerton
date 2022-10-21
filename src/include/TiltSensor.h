#include "util.h"

class TiltSensor(){
    public:
    void sensorSetup();
    TiltState getTilt(); //returns the current tilt state
    enum TiltState{
        HORIZONTAL = 0,
        TILT_UP = 1,
        TILT_DOWN = 2,
    }
}