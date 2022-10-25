#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include "include/util.h"
#include "include/Motorcontrol.h"
//#include "include/WifiDebug.h"


void MotorControl::MotorSetup() {
  //setup motors
  AFMS = Adafruit_MotorShield(); 
  motorL = AFMS.getMotor(LEFT_MOTOR_NUM);
  motorR = AFMS.getMotor(RIGHT_MOTOR_NUM);
  AFMS.begin();
  //set initial speed and direction
  SetMotors(0,0,FORWARD,FORWARD);
}

void MotorControl::ResetState(){
  LineState.status=LINE_ALIGNED;
  LineState.lost_line_counter=0;
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
  lmotor*=0.886;
  motorL->run(ldirection);
  motorR->run(rdirection);
  motorL->setSpeed(lmotor);
  motorR->setSpeed(rmotor);
}

bool MotorControl::LineFollowUpdate(double correction, bool LineDetected,WifiDebug Debug,bool forceFind=false){
  if(!LineDetected){
    LineState.lost_line_counter+=1;
  }else{
    LineState.lost_line_counter=0;
  }
  if(forceFind||(LineState.lost_line_counter>150 && LineState.status == LINE_ALIGNED) || (LineDetected && LineState.status == LINE_UNDETECTABLE)){ // if just lost the line, start scanning in the most likely direction. Or if just rediscovered line, start aligning
    LineState.status=INITIAL_SCAN;
    
    if(correction>=0){
      LineState.scan_direction=1;
      Debug.SendMessage("Lost line, scanning right");
    } else {
      LineState.scan_direction=0;
      Debug.SendMessage("Lost line, scanning left");
    }
  }
  // Make a movement depending on the state in LineState
  if(LineState.status==LINE_ALIGNED){ //if robot is on the line
    //set motor speeds based on correction steering value, used for line following
    int left_motor=(correction*LINE_FOLLOW_MOTOR_SWING)+LINE_FOLLOW_MOTOR_SPEED;
    int right_motor =(-correction*LINE_FOLLOW_MOTOR_SWING)+LINE_FOLLOW_MOTOR_SPEED;
    SetMotors(left_motor,right_motor);  
  } else if(LineState.status==INITIAL_SCAN){
    if(TurnSetAngle(90,LineState.scan_direction)==COMPLETE){//if turn complete, scan in other direction
    Debug.SendMessage("Scanning reverse");
      LineState.status=REVERSE_SCAN;
      LineState.scan_direction=!LineState.scan_direction;//reverse direction
    }
    if(LineDetected){
      Debug.SendMessage("Moving onto line");
      LineState.status=MOVING_ONTO_LINE;
      ismoving=0;
    }
  } else if(LineState.status==REVERSE_SCAN){
    if(TurnSetAngle(180,LineState.scan_direction)==COMPLETE){
      LineState.status=LINE_UNDETECTABLE;
      SetMotors(0,0);//stop robot if line cannot be found.
    }
    if(LineDetected){
      LineState.status=MOVING_ONTO_LINE;
      Debug.SendMessage("Moving onto line");
      ismoving=0;
    }
  } else if(LineState.status==MOVING_ONTO_LINE){ //move onto the line, so the rotation point is where the line was detected
    if(MoveSetDistance(DISTANCE_TO_ROTATION_POINT+1)==COMPLETE){
      Debug.SendMessage("aligning");
      LineState.status=ALIGN_SCAN;//scan backwards for the line to align the robot along it
      LineState.scan_direction=!LineState.scan_direction;
    }
  } else if(LineState.status==ALIGN_SCAN){
    if(TurnSetAngle(90,LineState.scan_direction)==COMPLETE){
      Debug.SendMessage("reverse aligning");
      LineState.status=REVERSE_ALIGN_SCAN;
      LineState.scan_direction=!LineState.scan_direction; //sweep in other direction
    }
    if(LineDetected){
      Debug.SendMessage("aligned!");
      LineState.status=LINE_ALIGNED; // robot is now fully aligned
      SetMotors(0,0);  
      LineState.lost_line_counter=0;
      ismoving=0;
    }
    
  } else if(LineState.status==REVERSE_ALIGN_SCAN){
    if(TurnSetAngle(180,LineState.scan_direction)==COMPLETE){
      LineState.status=LINE_UNDETECTABLE;
      SetMotors(0,0);//stop robot if line cannot be found
    }
    if(LineDetected){
      Debug.SendMessage("aligned!");
      LineState.status=LINE_ALIGNED; //robot is now fully aligned
      SetMotors(0,0);  
      LineState.lost_line_counter=0;
      ismoving=0;
    }
    
  }
  //if line cannot be found, return false
  if(LineState.status==LINE_UNDETECTABLE){
    return false;
  } else {
    return true;
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
    
    if (isclockwise == 1) {
      SetMotors(Sweep_Speed, Sweep_Speed, FORWARD, BACKWARD);
    }
    if (isclockwise == 0) {
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
  float time = distance/Measured_Speed; //calculate time to move 
  time -=Distance_Constant/Measured_Speed; //account for overshoot / undershoot
  return int(time);
}

int MotorControl::AngleCon(int angle){
  float time = angle/Measured_Turn_Rate;//calculate time it will take to turn
  time -=Angle_Constant/Measured_Turn_Rate; //account for overshoot due to momentum
  return int(time);
}

int MotorControl::TimeToAngleCon(int time){
  float ang = ang * Measured_Turn_Rate;//calculate angle it has moved through
  ang += Angle_Constant/2;//account for initial lag when starting to move
  return int(ang);

}