#include "include/TiltSensor.h"
#include <Arduino_LSM6DS3.h>

void TiltSensor::sensorSetup(){
    !IMU.begin();
    return;
}

void TiltSensor::reset(){
    for (int i = 0; i < TILT_AVERAGE_READINGS; i++){
        previous_x_readings[i] = 0;
        lastState=HORIZONTAL;
        buffer=0;
    }
}
TiltSensor::TiltState TiltSensor::getTilt(int dt_ms=0){//dt_ms in miliseconds. 0 if not taking a reading
    if(dt_ms==0){
        return lastState;
    }
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
    if (lastState == HORIZONTAL && buffer==0){
        if (x_average > GYRO_THRESHOLD){
            lastState = TILT_UP;
            buffer = GYRO_BUFFER;
        }
        if (x_average < -GYRO_THRESHOLD){
            lastState = TILT_DOWN;
            buffer = GYRO_BUFFER;

        }
    }
    if (lastState == TILT_UP && buffer==0){
        if (x_average < -GYRO_THRESHOLD){
            lastState = HORIZONTAL;
            buffer = GYRO_BUFFER;
        }
    }
    if (lastState == TILT_DOWN && buffer==0){
        if (x_average > GYRO_THRESHOLD){
            lastState = HORIZONTAL;                 
            buffer = GYRO_BUFFER;
        }
    }
    return lastState;
}