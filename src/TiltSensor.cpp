/*
TiltSensor.cpp
See header file for description
*/

#include "include/TiltSensor.h"
#include <Arduino_LSM6DS3.h>


//setup IMU
void TiltSensor::sensorSetup(){
    !IMU.begin();
    return;
}

//Reset state to horizontal
void TiltSensor::reset(){
    for (int i = 0; i < TILT_AVERAGE_READINGS; i++){
        previous_y_readings[i] = 0;
        lastState=HORIZONTAL;
        buffer=0;
    }
}

//Get the current tilt state
TiltSensor::TiltState TiltSensor::getTilt(int dt_ms=0){//dt_ms in miliseconds. 0 if not taking a reading
    if(dt_ms==0){//if no time elapsed, just return last state
        return lastState;
    }
    //take measurement from IMU
    float x, y, z;
    if (IMU.gyroscopeAvailable()) {
        x, y, z =IMU.readGyroscope(x, y, z);
    }
    //calculate the average y reading
    y_average=y;
    for (int i = TILT_AVERAGE_READINGS -1; i > 0; i--){
        previous_y_readings[i] = previous_y_readings[i - 1];
        y_average+=previous_y_readings[i];
    }
    previous_y_readings[0]=y;
    y_average/=TILT_AVERAGE_READINGS;
    //decrement buffer
    buffer-=dt_ms;
    if(buffer<0) buffer=0;
    //if the buffer has elapsed and the IMU value is above threshold, change state
    if (lastState == HORIZONTAL && buffer==0){
        if (y_average < -GYRO_THRESHOLD){
            lastState = TILT_UP;
            buffer = GYRO_BUFFER;
        }
        if (y_average > GYRO_THRESHOLD){
            lastState = TILT_DOWN;
            buffer = GYRO_BUFFER;
        }
    }
    if (lastState == TILT_UP && buffer==0){
        if (y_average > GYRO_THRESHOLD){
            lastState = HORIZONTAL;
            buffer = GYRO_BUFFER;
        }
    }
    if (lastState == TILT_DOWN && buffer==0){
        if (y_average < -GYRO_THRESHOLD){
            lastState = HORIZONTAL;                 
            buffer = GYRO_BUFFER;
        }
    }
    return lastState;
}