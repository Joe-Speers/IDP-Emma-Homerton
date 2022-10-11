/*
This program uses PID control to follow a line.
See Linear Systems and Control handout 6, page 14 onwards for PID control info. Effectivly uses the derivitive and integral of the error (in proportions DIFFERENTIAL_K and INTEGRAL_K)
 as well as proportional control (PROPORTIONAL_K) to set the correcting value.
*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//pin selection
int LEFT_MOTOR_NUM = 1;
int RIGHT_MOTOR_NUM = 2;
int LINE_SENSOR_PIN = A0;

//PID control constants
float PROPORTIONAL_K=1;
float INTEGRAL_K=7;
float DIFFERENTIAL_K=0.003;
float INTEGRAL_LIMIT=(1/INTEGRAL_K)/6;// hard limit on integral size

//sets the main loop() speed.
float dt=0.001;

//Settings for line sensor pin
double LINE_SENSE_MIDDLE = 531; //what reading should be treated as the 'LINE_SENSE_MIDDLE' of the line
int LINE_SENSE_MAX_AMPLITUDE = 500; // aproximate max +- possible reading about LINE_SENSE_MIDDLE. Range is therefore 2*LINE_SENSE_MAX_AMPLITUDE
double ERROR_DEAD_SPOT=0.01; //fraction of reading (between 0 and 1) to discard. e.g. 0.01 means if reading within 1% of 0 then treat as 0.

//motor turning settings
int MOTOR_SPEED = 200;//speed when 'correction' is zero (between 0 and 255)
int MOTOR_SWING = 200;//amount to swing from 'MOTOR_SPEED' as 'correction' varies. probably should be as big as speed

//motor variables
Adafruit_MotorShield AFMS;
Adafruit_DCMotor *motorL;
Adafruit_DCMotor *motorR;

//global variables
float integral=0; //stores the integral of 'error'
float error_array[5]; // stores the last 5 values of 'error', used to calculate differential


void setup() {
  //setup motors
  AFMS = Adafruit_MotorShield(); 
  motorL = AFMS.getMotor(LEFT_MOTOR_NUM);
  motorR = AFMS.getMotor(RIGHT_MOTOR_NUM);
  AFMS.begin();
  //set initial speed and direction
  motorL->setSpeed(150);// 0 to 255
  motorL->run(FORWARD);
  motorR->setSpeed(150);
  motorR->run(FORWARD);
  pinMode(LINE_SENSOR_PIN,INPUT);

  //start serial to pc (optional)
  Serial.begin(115200);
}

void loop() {
  //delay dt amount. NOTE: if using serial.println within loop() then this will take a number of miliseconds to transfer data to a pc! So this may not be accurate.
  delay(dt*1000);
  //take reading of line sensor and calculate error
  float diff = analogRead(LINE_SENSOR_PIN);
  double error = (LINE_SENSE_MIDDLE-diff)/LINE_SENSE_MAX_AMPLITUDE; // normalises reading to approximatly -1 to 1.
  //removes error if close to LINE_SENSE_MIDDLE
  if(error==0 or (error<=ERROR_DEAD_SPOT and error>=-ERROR_DEAD_SPOT)){
    error=0;
  }
  //add error to array of readings and shuffles the rest up one spot
  for (int i = 4; i > 0; i--){
		error_array[i] = error_array[i - 1];
  }
  error_array[0]=error;
  //// ### PID CONTROL ###
  //Differential: calculate slope of error based on the average slope of the last few readings.
  float differential=0;
  for(int i=0;i<4;i++){
    differential+=error_array[i+1]-error_array[i];
  }
  differential=differential/(4*dt);
  //Integral: updates the integral of the error
  integral+=error*dt;
  if(integral>INTEGRAL_LIMIT) integral=INTEGRAL_LIMIT;
  if(integral<-INTEGRAL_LIMIT) integral=-INTEGRAL_LIMIT;
  //Proportional is just -k * error
  // Calculate correction value
  double correction=(-PROPORTIONAL_K*error) + (-INTEGRAL_K*integral) +(-DIFFERENTIAL_K*differential);
  //chop off anything above 1 or below -1.
  Serial.println(correction);
  if(correction>1){
    correction=1;
  }
  if(correction<-1){
    correction=-1;
  }
  //print out any useful value here
  
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
