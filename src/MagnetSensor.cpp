#include <Wire.h>
#include "include/MagnetSensor.h"
#include "include/util.h"

void MagnetSensor::sensorSetup(){
    pinMode(MAGNET_SENSOR_PIN,INPUT);
}

bool MagnetSensor::MagnetDetected(){//must be called constatly
    if (digitalRead(MAGNET_SENSOR_PIN) == LOW)
    {
        detected_magnet_countdown-=1;
        
    }
    if (digitalRead(MAGNET_SENSOR_PIN) == HIGH)
    {
        detected_magnet_countdown=500;
    }
    if(detected_magnet_countdown<0)detected_magnet_countdown=0;
    if(detected_magnet_countdown>0){
        return true;
    } else {
        return false;
    }
}
