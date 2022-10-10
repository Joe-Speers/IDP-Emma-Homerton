#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
float k=1;
float integral_k=7;
float integral_limit=(1/integral_k)/6;
float differential_k=0.003;
float integral=0;
float error_array[5];
float dt=0.001;
float error;
int swing = 200;
int speed = 200;

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
  delay(dt*1000);
  // put your main code here, to run repeatedly:
  float diff = analogRead(diff_pin);
  
  double error = (middle-diff)/plus_minus;
  if(error==0 or (error<=dead_spot and error>=-dead_spot)){
    error=0;
  }
  for (int i = 4; i > 0; i--){
		error_array[i] = error_array[i - 1];
  }
  error_array[0]=error;
  float differential=0;
  for(int i=0;i<4;i++){
    differential+=error_array[i+1]-error_array[i];
  }
  differential=differential/(4*dt);
  integral+=error*dt;
  if(integral>integral_limit) integral=integral_limit;
  if(integral<-integral_limit) integral=-integral_limit;
  double correction=(-k*error) + (-integral_k*integral) +(-differential_k*differential);
  Serial.println(integral*1000);
  if(correction>1){
    correction=1;
  }
  if(correction<-1){
    correction=-1;
  }
  
  int left_motor=(correction*swing)+speed;
  int right_motor =(-correction*swing)+speed;
  if(left_motor>255) left_motor=255;
  if(right_motor>255) right_motor=255;
  if(left_motor<0) left_motor=0;
  if(right_motor<0) right_motor=0;
  motorL->setSpeed(left_motor);
  motorR->setSpeed(right_motor);
}
