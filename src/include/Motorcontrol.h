/*
MotorControl interfaces with the 2 DC motors and servo motor. Any actions peformed on the motors will persist untill changed again.
MotorControlUpdate() contains an algorithm to steer the robot based on a 'correction' value between -1 an 1, used for line follwing.
DistanceCon takes distance input in cm and returns millisecond value to move that distance.
AngleCon takes angle input and returns millisecond value to turn that amount.
TimeToAngleCon takes millisecond input and returns turned in that time, assumes angle turned in acceleration = angle turned in deceleration.
TODO: servo motor control and exact distance traveling.
*/
#include <Adafruit_MotorShield.h>

class MotorControl{
    public:
        void MotorSetup(); //Setup call to initilise motors
        void SetMotors(int lmotor, int rmotor, int ldirection=FORWARD,int rdirection=FORWARD); //Set motor speed and direction (optional)
        void MotorControlUpdate(double correction); //steers and drives the robot based on the correction input for line following
        int DistanceCon(int distance);
        int AngleCon(int angle);
        int TimeToAngleCon(int millisec);

    private:
        //motor objects
        Adafruit_MotorShield AFMS;
        Adafruit_DCMotor *motorL;
        Adafruit_DCMotor *motorR;
        //time variable
        int time;
        //angle variable
        int ang;
};
