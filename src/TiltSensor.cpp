#include "include/TiltSensor.h"
#include <Arduino_LSM6DS3.h>

void TiltSensor::sensorSetup(){
    !IMU.begin();
    return;
}

TiltSensor::TiltState TiltSensor::getTilt(int dt_ms){//dt_ms in miliseconds
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(x, y, z);
    }
    x_average=x;
    for (int i = TILT_AVERAGE_READINGS -1; i > 0; i--){
        previous_x_readings[i] = previous_x_readings[i - 1];
        x_average+=previous_x_readings[i];
    }
    previous_x_readings[0]=x;
    x_average/=TILT_AVERAGE_READINGS;
    buffer-=dt_ms;
    if(buffer<0) buffer=0;
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(x, y, z);
        
        if (lastState == HORIZONTAL && buffer==0){
            if (x_average > GYRO_THRESHOLD){
                lastState = TILT_UP;
                buffer = 400;
            }
            if (x_average < -GYRO_THRESHOLD){
                lastState = TILT_DOWN;
                buffer = 400;

            }
        }
        if (lastState == TILT_UP && buffer==0){
            if (x_average < -GYRO_THRESHOLD){
                lastState = HORIZONTAL;
                buffer = 400;
            }
        }
        if (lastState == TILT_DOWN && buffer==0){
            if (x_average > GYRO_THRESHOLD){
                lastState = HORIZONTAL;                 
                buffer = 400;
            }
        }
    } 
    return lastState;
}