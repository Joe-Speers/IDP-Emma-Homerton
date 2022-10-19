#include "include/TiltSensor.h"
#include <Arduino_LSM6DS3.h>

void sensorSetup(){
    !IMU.begin();
    return;
}

TiltState TiltSensor::getTilt(int dt){
    //todo get tilt state
    buffer -= dt;
    if (buffer <= 0){
        if (IMU.gyroscopeAvailable()) {
            IMU.readGyroscope(x, y, z);
            
            if (TiltState == HORIZONTAL){
                if (x > GYRO_THRESHOLD){
                    TiltState = TILT_UP
                    buffer = 100;
                }
                if (x < GYRO_THRESHOLD){
                    TiltState = TILT_DOWN
                    buffer = 100;

                }
            }
            if (TiltState == TILT_UP){
                if (x < GYRO_THRESHOLD){
                    TiltState = HORIZONTAL
                    buffer = 100;
                }
            }
            if (TiltState == TILT_DOWN){
                if (x > GYRO_THRESHOLD){
                    TiltState = HORIZONTAL                    
                    buffer = 100;
                }
            }
        }
    }  

}