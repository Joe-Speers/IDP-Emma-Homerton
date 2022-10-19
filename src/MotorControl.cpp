#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include "include/util.h"
#include "include/Motorcontrol.h"


void MotorControl::MotorSetup() {
  //setup motors
  AFMS = Adafruit_MotorShield(); 
  motorL = AFMS.getMotor(LEFT_MOTOR_NUM);
  motorR = AFMS.getMotor(RIGHT_MOTOR_NUM);
  AFMS.begin();
  //set initial speed and direction
  SetMotors(0,0,FORWARD,FORWARD);
}

void MotorControl::SetMotors(int lmotor, int rmotor, int ldirection=FORWARD,int rdirection=FORWARD){
  // restrict input to valid range
  if(lmotor>255) lmotor=255;
  if(rmotor>255) rmotor=255;
  if(lmotor<0) lmotor=0;
  if(rmotor<0) rmotor=0;
  //set motor speed and direction
  motorL->run(ldirection);
  motorR->run(rdirection);
  motorL->setSpeed(lmotor);
  motorR->setSpeed(rmotor);
}

void MotorControl::MotorControlUpdate(double correction){
  //set motor speeds based on correction steering value, used for line following
  int left_motor=(correction*LINE_FOLLOW_MOTOR_SWING)+LINE_FOLLOW_MOTOR_SPEED;
  int right_motor =(-correction*LINE_FOLLOW_MOTOR_SWING)+LINE_FOLLOW_MOTOR_SPEED;
  SetMotors(left_motor,right_motor);  
}

bool MotorControl::MoveSetDistance(int distance, int s, int m){
  
  milli = (s + (m *1000));
  if (ismoving == 0){
    ismoving = 1;
    starttime = milli;
    SetMotors(Default_Speed, Default_Speed);
    stoptime = starttime + DistanceCon(distance);
  }

  if (milli >= stoptime){
    SetMotors(0,0);
    ismoving = 0;

    return ismoving;
  }

}

void MotorControl::TurnSetAngle(int angle, int s, int m){

  milli = (s + (m *1000));
    if (ismoving == 0){
      ismoving = 1;
      starttime = milli;
      SetMotors(Default_Speed, Default_Speed, FORWARD, BACKWARD);
      stoptime = starttime + AngleCon(angle);
    }

    if (milli >= stoptime){
      SetMotors(0,0);
      ismoving = 0;

      return ismoving;
    }

  
}

int MotorControl::DistanceCon(int distance){

  distance -= Distance_Acceleration;
  time = int(distance*Distance_To_Time);
  time += Time_Accelartionn;
  
  return time;

}

int MotorControl::AngleCon(int angle){

  angle -= Angle_Acceleration;
  time = int(angle*Angle_To_Time);
  time += Time_Angular_Acceleration;
  
  return time;

}

int MotorControl::TimeToAngleCon(int time){

  time -= int (Time_Angular_Acceleration /2);
  ang = int(ang / Angle_To_Time);
  ang += Time_Angular_Acceleration;
  
  return ang;

}