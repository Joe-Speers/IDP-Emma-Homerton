/*
To use this program go to Tools > Serial plotter (NOT serial monitor) in the arduino IDE. 
Position the robot so the sensors are right of the line.
Then press reset button and it should plot a graph as it rotates accross the line (for 1 second). This can be used to calibrate the sensor
*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//pin assignment
int left_motor_num = 1;
int right_motor_num = 2;
int diff_pin = A0;
//motor speed, set as low as possible, but high enought for the robot to actually move
int mot_speed=90;

//motor variables
Adafruit_MotorShield AFMS;
Adafruit_DCMotor *motorL;
Adafruit_DCMotor *motorR;

void setup() {
  //setup motors
  AFMS = Adafruit_MotorShield(); 
  motorL = AFMS.getMotor(left_motor_num);
  motorR = AFMS.getMotor(right_motor_num);
  AFMS.begin();
  motorL->setSpeed(0);// 0 to 255
  motorL->run(FORWARD);
  motorR->setSpeed(0);
  motorR->run(BACKWARD);
  pinMode(diff_pin,INPUT);
  //setup serial
  Serial.begin(9600);
  //wait for 3 seconds
  delay(3000);
  Serial.println("start");
  //tell motors to rotate about axel
  motorL->setSpeed(mot_speed);// 0 to 255
  motorL->run(FORWARD);
  motorR->setSpeed(mot_speed);
  motorR->run(BACKWARD);
}

int i=0;//counter
void loop() {
  if(i<500){ // take 500 readings (max that fits on the graph)
    float line_sense = analogRead(diff_pin);
    Serial.println(line_sense);//print out reading. This will plot it on the graph
    i++;
  } else{
    //stop motors
    motorL->setSpeed(0);
    motorL->run(FORWARD);
    motorR->setSpeed(0);
    motorR->run(BACKWARD);
  }
}
