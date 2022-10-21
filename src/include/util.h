/* 
util.h
Contains key global constants used throughout the modules, including pin assignments.
Also contains enumerables for the state system.
*/
#pragma once

// ### constants ###

//Motor speeds
#define Default_Speed 255
#define Sweep_Speed 100

//distance between line sensors and turning point
#define DISTANCE_TO_ROTATION_POINT 16

//Gyroscope Constants
#define GYRO_THRESHOLD 80
#define GYRO_BUFFER 100

//Block gathering constants
#define GAP_LEFT_TO_BLOCK 5
#define MIN_WALL_DISTANCE 80
#define ARMS_CLOSED_ANGLE 90 //do not know these values currently, depends on setup and gearing
#define ARMS_OPEN_ANGLE 0

//Turning direction 
#define CLOCKWISE 1
#define ANTI_CLOCKWISE 0

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

#define COMPLETE 0 //value for SetDistance and SetAngle once complete

//Settings for line sensor value normalisation
#define LINE_SENSE_MIDDLE           531 //what reading should be treated as the 'middle' of the line
#define LINE_SENSE_MAX_AMPLITUDE    500 // aproximate max +- possible reading about LINE_SENSE_MIDDLE. Range is therefore 2*LINE_SENSE_MAX_AMPLITUDE
#define ERROR_DEAD_SPOT             0.02 //fraction of reading (between 0 and 1) to discard. e.g. 0.01 means if reading within 1% of 0 then treat as 0.

// ### Pin assignment ###

#define LINE_SENSOR_PIN         A0

#define JUNCTION_SENSOR_PIN 2
#define ULTRASOUND_TRIGGER_PIN 9
#define ULTRASOUND_ECHO_PIN 10
#define SERVO_PIN 5
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