#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
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

void MotorControl::ServoSetup(){
  // Attach the Servo variable to a pin:
  myservo.attach(SERVO_PIN);
}

void MotorControl::SetServoAngle(int angle){
  //restrict input to valid range
  if (angle < 0 ){angle = 0;}
  if (angle > 180){angle = 180;}
  //set servo angle
  myservo.write(angle);
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

void MotorControl::LineFollowUpdate(double correction, bool onLine){
  if(LineStatus.LINE_DETECTED){
    if(!MoveSetDistance(DISTANCE_TO_ROTATION_POINT)){
      //now aligned with line, just need to rotate onto line
    }
  }
  if(onLine){
    if(LineStatus.ON_LINE){

    } else {
      //line detected!
      LineStatus.LINE_DETECTED;
      MoveSetDistance(DISTANCE_TO_ROTATION_POINT);
    }
  } else {
    if(correction>0){
      LineStatus=RIGHT_SWEEP;
      TurnSetAngle(90,true);
    } else {
      LineStatus = LEFT_SWEEP;
    }
  }
  else{
    if(Line)
  }
  if(!LostLine){ //if robot is on the line
    //set motor speeds based on correction steering value, used for line following
    int left_motor=(correction*LINE_FOLLOW_MOTOR_SWING)+LINE_FOLLOW_MOTOR_SPEED;
    int right_motor =(-correction*LINE_FOLLOW_MOTOR_SWING)+LINE_FOLLOW_MOTOR_SPEED;
    SetMotors(left_motor,right_motor);  
    return;
  } else {
    
  }
}

bool MotorControl::MoveSetDistance(int distance){
  
  milli = millis();

  if (ismoving == 0){
    ismoving = 1;
    starttime = milli;
    SetMotors(Default_Speed, Default_Speed);
    stoptime = starttime + DistanceCon(distance);
  }

  if (milli >= stoptime){
    SetMotors(0,0);
    ismoving = 0;
  }
  return ismoving;
}

bool MotorControl::TurnSetAngle(int angle, bool isclockwise){

  milli = millis();
    if (ismoving == 0){
      ismoving = 1;
      starttime = milli;
      
      if (isclockwise = 1) {
        SetMotors(Sweep_Speed, Sweep_Speed, FORWARD, BACKWARD);
      }
      if (isclockwise = 0) {
        SetMotors(Sweep_Speed, Sweep_Speed, BACKWARD, FORWARD);
      }

      stoptime = starttime + AngleCon(angle);
    }

    if (milli >= stoptime){
      SetMotors(0,0);
      ismoving = 0;
    }
  return ismoving;
  
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