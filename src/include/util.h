/* 
util.h
Contains key global constants used throughout the modules, including pin assignments.
Also contains enumerables for the state system.
*/
#pragma once

// ### constants ###

//PID constants
#define DEFAULT_PROPORTIONAL_K 1.1
#define DEFAULT_INTEGRAL_K 1.0
#define DEFAULT_DERIVATIVE_K 0.01
#define DEFAULT_INTEGRAL_CORRECTION_LIMIT 0.3 //limits the size of the integral.

// line follow motor settings
#define LINE_FOLLOW_MOTOR_SPEED     255 //speed when 'correction' is zero (between 0 and 255)
#define LINE_FOLLOW_MOTOR_SWING     255 //amount to swing from 'MOTOR_SPEED' as 'correction' varies. probably should be as big as speed

//Settings for line sensor value normalisation
#define LINE_SENSE_MIDDLE           531 //what reading should be treated as the 'middle' of the line
#define LINE_SENSE_MAX_AMPLITUDE    500 // aproximate max +- possible reading about LINE_SENSE_MIDDLE. Range is therefore 2*LINE_SENSE_MAX_AMPLITUDE
#define ERROR_DEAD_SPOT             0.01 //fraction of reading (between 0 and 1) to discard. e.g. 0.01 means if reading within 1% of 0 then treat as 0.

// ### Pin assignment ###

#define LINE_SENSOR_PIN         A0
// Motor shield motor numbers (1-4)
#define LEFT_MOTOR_NUM          1
#define RIGHT_MOTOR_NUM         2


// ### enums for State system ###

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


enum Purpose{ //Describes the current goal
    EXIT_START_BOX = 0,
    TRAVEL_TO_FAR_SIDE = 1, //add more as needed

};

enum Task{  //Describes the exact task the robot is peforming
    MOVE_FORWARD = 0,
    REVERSE = 1,
    TURN_LEFT = 2,
    TURN_RIGHT = 3,
    FOLLOW_LINE = 4,
    SLOW_SWEEP =5, //add more as needed

};

