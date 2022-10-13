#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include "include/start.h"

int start::startmovement(int s, int m){
    
    //Hardcoded millissecon values for start timings
    if (s > 10 &&  s < 11){
        
        right_motor = 200;
        left_motor = 200;
    
    }
    if (s > 12 &&  s < 13){

        right_motor = 200;
        left_motor = 0;

    }
    return left_motor, right_motor;
}
