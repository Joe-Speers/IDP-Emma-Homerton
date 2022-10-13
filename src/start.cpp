#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include "include/start.h"

int start::startmovement(int ms){
    
    //Hardcoded millissecon values for start timings
    if (ms > (double)10000 &&  ms < (double)11000){
        
        right_motor = 200;
        left_motor = 200;
    
    }
    if (ms > (double)12000 &&  ms < (double)13000){

        right_motor = 200;
        left_motor = 0;

    }
    return left_motor, right_motor;
}
