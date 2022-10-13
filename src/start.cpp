#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include "include/MotorControl.h"
#include "include/WifiDebug.h"
#include "include/start.h"

void start::startmovement(int s, int m, MotorControl Mcon, WifiDebug Debug){
    
    //Hardcoded millissecon values for start timings
    if(s<10){
        right_motor = 0;
        left_motor = 0;
    }
    if (s = 10){
        if(m=0) Debug.SendMessage("moving forward");
        right_motor = 200;
        left_motor = 200;
    }
    if (s =12){
        if(m=0) Debug.SendMessage("turning right");
        right_motor = 200;
        left_motor = 0;
    }
    Mcon.SetMotors(left_motor,right_motor);
}
