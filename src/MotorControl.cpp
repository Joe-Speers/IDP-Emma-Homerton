#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include "include/Motorcontrol.h"


//pin selection
int LEFT_MOTOR_NUM = 1;
int RIGHT_MOTOR_NUM = 2;

//motor variables
Adafruit_MotorShield AFMS;
Adafruit_DCMotor *motorL;
Adafruit_DCMotor *motorR;

void MotorControl::MotorSetup() {
  //setup motors
  AFMS = Adafruit_MotorShield(); 
  motorL = AFMS.getMotor(LEFT_MOTOR_NUM);
  motorR = AFMS.getMotor(RIGHT_MOTOR_NUM);
  AFMS.begin();
  //set initial speed and direction
  motorL->setSpeed(0);// 0 to 255
  motorL->run(FORWARD);
  motorR->setSpeed(0);
  motorR->run(FORWARD);
}

void MotorControl::SetMotors(int lmotor, int rmotor, int ldirection=FORWARD,int rdirection=FORWARD){
  
  if(lmotor>255) lmotor=255;
  if(rmotor>255) rmotor=255;
  if(lmotor<0) lmotor=0;
  if(rmotor<0) rmotor=0;
  //set motor speed
  motorL->run(ldirection);
  motorR->run(rdirection);
  motorL->setSpeed(lmotor);
  motorR->setSpeed(rmotor);

}

void MotorControl::MotorControlUpdate(double correction){
  //set motor speeds based on correction value.
  int left_motor=(correction*MOTOR_SWING)+MOTOR_SPEED;
  int right_motor =(-correction*MOTOR_SWING)+MOTOR_SPEED;
  SetMotors(left_motor,right_motor);  
}
