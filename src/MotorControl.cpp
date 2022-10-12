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

void MotorCon::MotorSetup() {
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

  if(left_motor>255) left_motor=255;
  if(right_motor>255) right_motor=255;
  if(left_motor<0) left_motor=0;
  if(right_motor<0) right_motor=0;
  //set motor speed
  motorL->setSpeed(left_motor);
  motorR->setSpeed(right_motor);

}

void MotorCon::MotorUpdate(int lmotor, int rmotor){
  
  if(lmotor>255) lmotor=255;
  if(rmotor>255) rmotor=255;
  if(lmotor<0) lmotor=0;
  if(rmotor<0) rmotor=0;
  //set motor speed
  motorL->setSpeed(lmotor);
  motorR->setSpeed(rmotor);

}

void MotorCon::MotorControlUpdate(double correction){

  //set motor speeds based on correction value.
  int left_motor=(correction*MOTOR_SWING)+MOTOR_SPEED;
  int right_motor =(-correction*MOTOR_SWING)+MOTOR_SPEED;
  //ensures motor speed is within valid range
  if(left_motor>255) left_motor=255;
  if(right_motor>255) right_motor=255;
  if(left_motor<0) left_motor=0;
  if(right_motor<0) right_motor=0;
  //set motor speed
  motorL->setSpeed(left_motor);
  motorR->setSpeed(right_motor);
  
}
