#include <Adafruit_MotorShield.h>
class MotorControl{
    public:
        //motor turning settings
        int MOTOR_SPEED = 200;//speed when 'correction' is zero (between 0 and 255)
        int MOTOR_SWING = 200;//amount to swing from 'MOTOR_SPEED' as 'correction' varies. probably should be as big as speed

        void MotorSetup(); //Setup call to initilise sensors
        void SetMotors(int lmotor, int rmotor, int ldirection=FORWARD,int rdirection=FORWARD); //Set motor speed and direction
        void MotorControlUpdate(double correction); //steers and drives the robot based on the correction input

};

