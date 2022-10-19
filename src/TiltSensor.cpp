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
            
            if (lastState == HORIZONTAL){
                if (x > GYRO_THRESHOLD){
                    lastState = TILT_UP
                    buffer = 100;
                }
                if (x < GYRO_THRESHOLD){
                    lastState = TILT_DOWN
                    buffer = 100;

                }
            }
            if (lastState == TILT_UP){
                if (x < GYRO_THRESHOLD){
                    lastState = HORIZONTAL
                    buffer = 100;
                }
            }
            if (lastState == TILT_DOWN){
                if (x > GYRO_THRESHOLD){
                    lastState = HORIZONTAL                    
                    buffer = 100;
                }
            }
        }
    }  

}