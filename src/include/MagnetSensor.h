#include "util.h"

class MagnetSensor{
    public:
        void sensorSetup();
        bool MagnetDetected();
    private:
        int detected_magnet_countdown=0;
};