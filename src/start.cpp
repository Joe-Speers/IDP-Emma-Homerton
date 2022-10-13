#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include "include/start.h"

int start::startmovement(int s, int m){
    
    //Hardcoded s values for start timings
    if (s <2){
        right_motor = 200;
        left_motor = 200;
    }
    if (s <2){
        right_motor = 200;
        left_motor = 50;
    }

    return left_motor, right_motor;
}
