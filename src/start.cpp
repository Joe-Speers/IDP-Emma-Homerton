#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include "include/start.h"

int start::startmovement(int dt){
    
    double ms = dt/1000;
    //Hardcoded millissecon values for start timings
    if (ms <10000){
        right_motor = 200;
        left_motor = 200;
    }
    if (ms <11800){
        right_motor = 200;
        left_motor = 50;
    }

    return left_motor, right_motor;
}
