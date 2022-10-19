/*
MotorControl interfaces with the 2 DC motors and servo motor. Any actions peformed on the motors will persist untill changed again.
MotorControlUpdate() contains an algorithm to steer the robot based on a 'correction' value between -1 an 1, used for line follwing.
DistanceCon takes distance input in cm and returns millisecond value to move that distance.
AngleCon takes angle input and returns millisecond value to turn that amount.
TimeToAngleCon takes millisecond input and returns turned in that time, assumes angle turned in acceleration = angle turned in deceleration.
MoveSetDistance takes a distance value and runs motor for required time to move distance input, constantly returns a bool which will return 0 for movement complete.
TurnSetAngle takes a Angle value and runs motor for required time to turn angle input, constantly returns a bool which will return 0 for movement complete.
TODO: servo motor control and exact distance traveling.
*/
#include <Adafruit_MotorShield.h>

class MotorControl{
    public:
        void MotorSetup(); //Setup call to initilise motors
        void SetMotors(int lmotor, int rmotor, int ldirection=FORWARD,int rdirection=FORWARD); //Set motor speed and direction (optional)
        void MotorControlUpdate(double correction); //steers and drives the robot based on the correction input for line following
        bool MoveSetDistance(int distance);//moves set distance, returns bool of 0 when movement is complete
        bool TurnSetAngle(int angle, bool isclockwise);//turns set angle clockwise, returns bool of 0 when movement is complete
        int DistanceCon(int distance);//converts distance to time at default speed
        int AngleCon(int angle);//converts angles to time at default speed
        int TimeToAngleCon(int millisec);//converts time to angle to turn

    private:
        //motor objects
        Adafruit_MotorShield AFMS;
        Adafruit_DCMotor *motorL;
        Adafruit_DCMotor *motorR;
        //time variable
        int time;
        //angle variable
        int ang;
        //time elapsed in milliseconds
        int milli;
        //set movement state
        bool ismoving = 0;
        //setmovement's start time
        int starttime;
        //setmovement's stop time 
        int stoptime;

};
