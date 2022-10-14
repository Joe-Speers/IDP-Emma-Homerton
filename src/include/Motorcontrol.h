/*
MotorControl interfaces with the 2 DC motors and servo motor. Any actions peformed on the motors will persist untill changed again.
MotorControlUpdate() contains an algorithm to steer the robot based on a 'correction' value between -1 an 1, used for line follwing.

TODO: servo motor control and exact distance traveling.
*/
#include <Adafruit_MotorShield.h>

class MotorControl{
    public:
        void MotorSetup(); //Setup call to initilise motors
        void SetMotors(int lmotor, int rmotor, int ldirection=FORWARD,int rdirection=FORWARD); //Set motor speed and direction (optional)
        void MotorControlUpdate(double correction); //steers and drives the robot based on the correction input for line following
    private:
        //motor objects
        Adafruit_MotorShield AFMS;
        Adafruit_DCMotor *motorL;
        Adafruit_DCMotor *motorR;
};
