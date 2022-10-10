/*
To use this program go to Tools > Serial plotter in the arduino IDE. 
Position the robot right of the line.
Then press reset button and it should plot a graph 
*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

int left_motor_num = 1;
int right_motor_num = 2;
int diff_pin = A0;
int mot_speed=90;

Adafruit_MotorShield AFMS;
Adafruit_DCMotor *motorL;
Adafruit_DCMotor *motorR;

void setup() {
  AFMS = Adafruit_MotorShield(); 
  motorL = AFMS.getMotor(left_motor_num);
  motorR = AFMS.getMotor(right_motor_num);
  AFMS.begin();
  motorL->setSpeed(0);// 0 to 255
  motorL->run(FORWARD);
  motorR->setSpeed(0);
  motorR->run(BACKWARD);
  // put your setup code here, to run once:
  pinMode(diff_pin,INPUT);
  // start serial port at 9600 bps:
  Serial.begin(9600);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  delay(5000);
  Serial.println("start");
  motorL->setSpeed(mot_speed);// 0 to 255
  motorL->run(FORWARD);
  motorR->setSpeed(mot_speed);
  motorR->run(BACKWARD);
}

int i=0;
void loop() {
  // put your main code here, to run repeatedly:
  if(i<500){
    float diff = analogRead(diff_pin);
    Serial.println(diff);
    i++;
  } else{
    motorL->setSpeed(0);// 0 to 255
    motorL->run(FORWARD);
    motorR->setSpeed(0);
    motorR->run(BACKWARD);
  }
}
