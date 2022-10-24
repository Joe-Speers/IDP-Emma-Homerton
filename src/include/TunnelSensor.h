#include "util.h"

class TunnelSensor{
    public:
        void sensorSetup();
        bool TunnelDetected();
        bool WallCollisionLeft();
        bool WallCollisionRight();

};