/* 
util.h
Contains key global constants used throughout the modules, including pin assignments.
Also contains enumerables for the state system.
*/
#pragma once

// ### constants ###
#define DERIVITIVE_LINE_SENSE_THRESHOLD 3//3
#define PROPORTIONAL_LINE_SENSE_REGION 0.35//0.35
#define INTEGRAL_LINE_SENSE_REGION 0.01 //bound is currently 0.12
//Motor speeds
#define Default_Speed 255
#define Sweep_Speed 255

//distance between line sensors and turning point
#define DISTANCE_TO_ROTATION_POINT 17.9

//Gyroscope Constants
#define GYRO_THRESHOLD 5
#define GYRO_BUFFER 1000
#define TILT_AVERAGE_READINGS 50

//Block gathering constants
#define GAP_LEFT_TO_BLOCK 5
#define MIN_WALL_DISTANCE 80
#define ARMS_CLOSED_ANGLE 90 //do not know these values currently, depends on setup and gearing
#define ARMS_OPEN_ANGLE 0

//Turning direction 
#define CLOCKWISE 1
#define ANTI_CLOCKWISE 0

//PID constants
#define DEFAULT_PROPORTIONAL_K 4 //4
#define DEFAULT_INTEGRAL_K 5//5
#define DEFAULT_DERIVATIVE_K 0 // currently disabled
#define DEFAULT_INTEGRAL_CORRECTION_LIMIT 0.5 //0.5 limits the size of the integral.

// line follow motor settings
#define LINE_FOLLOW_MOTOR_SPEED     255 //speed when 'correction' is zero (between 0 and 255)
#define LINE_FOLLOW_MOTOR_SWING     255 //amount to swing from 'MOTOR_SPEED' as 'correction' varies. probably should be as big as speed

//Conversion constants
#define Distance_Constant 0.0917 //distance overshoot constant (in cm)
#define Measured_Speed 0.01529 //measured speed in cm/ms for conversion (at speed 255)

#define Angle_Constant -4.47 //angle overshoot constant (in degrees)
#define Measured_Turn_Rate 0.06915 //measured turn rate (in degrees/ms) 

#define COMPLETE 0 //value for SetDistance and SetAngle once complete

//Settings for line sensor value normalisation
#define LINE_SENSE_MIDDLE           531 //what reading should be treated as the 'middle' of the line
#define LINE_SENSE_MAX_AMPLITUDE    500 // aproximate max +- possible reading about LINE_SENSE_MIDDLE. Range is therefore 2*LINE_SENSE_MAX_AMPLITUDE
#define ERROR_DEAD_SPOT             0.02 //fraction of reading (between 0 and 1) to discard. e.g. 0.01 means if reading within 1% of 0 then treat as 0.

// ### Pin assignment ###

#define LINE_SENSOR_PIN         A0 //connection to subtraction circuit

#define JUNCTION_SENSOR_PIN 2
#define ULTRASOUND_TRIGGER_PIN 9
#define ULTRASOUND_ECHO_PIN 10
#define SERVO_PIN 5
#define AMBER_LED_PIN 8
// Motor shield motor numbers (1-4)
#define LEFT_MOTOR_NUM          2
#define RIGHT_MOTOR_NUM         1


// ### enums for State system ###
// ANY CHANGES TO THESE, please replicate in the top of pc_debug.py

enum Purpose{ //Describes the current goal
    EXIT_START_BOX = 0,
    TRAVEL_TO_FAR_SIDE = 1, //add more as needed
    PICK_UP_BLOCK = 2,
    TRAVEL_TO_START_SIDE = 3,
    DROP_BLOCK = 4,
    RETURN_HOME = 5,
};

enum Location{
    START_SQUARE            = 0,
    DROPOFF_SIDE            = 1,
    RAMP                    = 2,
    COLLECTION_SIDE         = 3,
    CROSS                   = 4,
    BLOCK_COLLECTION_AREA   = 5,
    TUNNEL                  = 6,
    RED_SQUARE              = 7,
    GREEN_SQUARE             = 8,
};

enum Task{  //Describes the exact task the robot is peforming
    STOPPED         = 0,
    MOVE_FORWARD    = 1,
    REVERSE         = 2,
    TURN_LEFT       = 3,
    TURN_RIGHT      = 4,
    FOLLOW_LINE     = 5,
    SLOW_SWEEP      = 6, //add more as needed

};