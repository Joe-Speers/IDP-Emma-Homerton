#include "util.h"

class TiltSensor{
    public:
        enum TiltState{
            HORIZONTAL = 0,
            TILT_UP = 1,
            TILT_DOWN = 2,
        };
    float x_average;
    void sensorSetup();
    void reset(); //reset to horizontal
    TiltState getTilt(int dt_ms); //returns the current tilt state
    float getRaw();
    private:
        TiltState lastState = HORIZONTAL;
        float x, y, z;
        float previous_x_readings[TILT_AVERAGE_READINGS];
        int buffer = 0;
};