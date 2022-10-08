#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
float k=1;

int left_motor_num = 1;
int right_motor_num = 2;
int diff_pin = A0;
double middle = 531;
int plus_minus = 500;
double dead_spot=0.01;

Adafruit_MotorShield AFMS;
Adafruit_DCMotor *motorL;
Adafruit_DCMotor *motorR;

void setup() {
  AFMS = Adafruit_MotorShield(); 
  motorL = AFMS.getMotor(left_motor_num);
  motorR = AFMS.getMotor(right_motor_num);
  AFMS.begin();
  motorL->setSpeed(150);// 0 to 255
  motorL->run(FORWARD);
  motorR->setSpeed(150);
  motorR->run(FORWARD);
  // put your setup code here, to run once:
  pinMode(diff_pin,INPUT);
  // start serial port at 9600 bps:
  Serial.begin(9600);

  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  float diff = analogRead(diff_pin);
  
  double error = (middle-diff)/plus_minus;
  if(error==0 or (error<=dead_spot and error>=-dead_spot)){
    error=0;
  }
  double correction=-k*error;
  if(correction>1){
    correction=1;
  }
  if(correction<-1){
    correction=-1;
  }
  Serial.println(correction);
  motorL->setSpeed(int(125+(correction*125)));
  motorR->setSpeed(int(125-(correction*125)));
}
