/* 
util.h
Contains key global constants used throughout the modules, including pin assignments.
Also contains enumerables for the state system.
*/
#pragma once

// ### constants ###

//PID constants
#define DEFAULT_PROPORTIONAL_K 0.8
#define DEFAULT_INTEGRAL_K 2.5
#define DEFAULT_DERIVATIVE_K 0 // currently disabled
#define DEFAULT_INTEGRAL_CORRECTION_LIMIT 0.3 //limits the size of the integral.

// line follow motor settings
#define LINE_FOLLOW_MOTOR_SPEED     255 //speed when 'correction' is zero (between 0 and 255)
#define LINE_FOLLOW_MOTOR_SWING     255 //amount to swing from 'MOTOR_SPEED' as 'correction' varies. probably should be as big as speed

//Conversion constants
#define Distance_Acceleration 2 //distance covered in motor acceleration and deceleration
#define Time_Accelartionn 50 //time taken for acceleration and deceleration
#define Distance_To_Time 10 //constant of proportionality between distance and time at constant speed

#define Angle_Acceleration 2 //degrees turned in angular acceleration and deceleration
#define Time_Angular_Acceleration 10 //time taken for angular acceleration and deceleration
#define Angle_To_Time 5 //constant of proportionality between angle and time at constant angular speed

//Settings for line sensor value normalisation
#define LINE_SENSE_MIDDLE           531 //what reading should be treated as the 'middle' of the line
#define LINE_SENSE_MAX_AMPLITUDE    500 // aproximate max +- possible reading about LINE_SENSE_MIDDLE. Range is therefore 2*LINE_SENSE_MAX_AMPLITUDE
#define ERROR_DEAD_SPOT             0.02 //fraction of reading (between 0 and 1) to discard. e.g. 0.01 means if reading within 1% of 0 then treat as 0.

// ### Pin assignment ###

#define LINE_SENSOR_PIN         A0
#define ULTRASOUND_TRIGGER_PIN 9
#define ULTRASOUND_ECHO_PIN 10
// Motor shield motor numbers (1-4)
#define LEFT_MOTOR_NUM          2
#define RIGHT_MOTOR_NUM         1


// ### enums for State system ###
// ANY CHANGES TO THESE, please replicate in the top of pc_debug.py

enum Purpose{ //Describes the current goal
    EXIT_START_BOX = 0,
    TRAVEL_TO_FAR_SIDE = 1, //add more as needed

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
    GREEN_SQARE             = 8
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