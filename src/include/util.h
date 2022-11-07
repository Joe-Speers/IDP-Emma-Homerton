/* 
util.h
Contains key global constants used throughout the modules, including pin assignments.
Also contains enumerables for the state system.
*/
#pragma once

// ###### KEY CONSTANTS ######

// ### IMU Constants ###
#define GYRO_THRESHOLD          5       //threshold for changing rotation state
#define GYRO_BUFFER             1000    //minimum delay (ms) between state changes
#define TILT_AVERAGE_READINGS   50      //number of readings to use for average

// ### Block Sweep constants ###
#define ACCURATE_MEASURING_DISTANCE             45 //Distance (cm) away from block where IR measurement is most accurate
#define DISTANCE_MEASURE_MAGNET                 2  //Distance to measure magnet from
#define ULTRASOUND_BLOCK_DETECTION_THRESHOLD    6  //Distance to switch from IR to ultrasound
#define MIN_WALL_DISTANCE                       60 //maximum distance to treat as a block detection
#define CROSS_OFFSET                            15 //distance robot will move from the cross so block is in IR range
#define PI                                      3.141592

// ### Motor Control ###
// Servo control
#define ARMS_CLOSED_ANGLE                       82 
#define ARMS_OPEN_ANGLE                         150
//Turning direction 
#define CLOCKWISE           1
#define ANTI_CLOCKWISE      0
//Motor speeds
#define Default_Speed       255 //Speed for set distance movements
#define Sweep_Speed         255 //Speed for sweeping for a block
//Conversion constants for distance movements
#define Distance_Constant   0.0917 //distance overshoot constant (in cm)
#define Measured_Speed      0.01529 //measured speed in cm/ms for conversion (at speed 255)
#define Angle_Constant      -4.47 //angle overshoot constant (in degrees)
#define Measured_Turn_Rate  0.06915 //measured turn rate (in degrees/ms) 
#define COMPLETE            0 //Indicates movement complete
//distance between line sensors and turning point in cm
#define DISTANCE_TO_ROTATION_POINT 17.9

// ### Line Following ###
//Settings for line sensor value normalisation
#define LINE_SENSE_MIDDLE           531 //what reading should be treated as the 'middle' of the line
#define LINE_SENSE_MAX_AMPLITUDE    500 // aproximate max +- possible reading about LINE_SENSE_MIDDLE. Range is therefore 2*LINE_SENSE_MAX_AMPLITUDE
#define ERROR_DEAD_SPOT             0.02 //fraction of reading (between 0 and 1) to discard. e.g. 0.01 means if reading within 1% of 0 then treat as 0.
//PID constants
#define DEFAULT_PROPORTIONAL_K              4
#define DEFAULT_INTEGRAL_K                  5
#define DEFAULT_DERIVATIVE_K                0 // currently disabled
#define DEFAULT_INTEGRAL_CORRECTION_LIMIT   0.5 //limits the size of the integral.
// Line Sensing constants (thresholds for when the robot thinks it is following the line)
#define DERIVITIVE_LINE_SENSE_THRESHOLD     2
#define PROPORTIONAL_LINE_SENSE_REGION      0.2
#define INTEGRAL_LINE_SENSE_REGION          0.03 
#define LINE_LOST_COUNT                     10 //number of loops where the line is lost before entering recovery
// line follow motor settings
#define LINE_FOLLOW_MOTOR_SPEED             255 //speed when 'correction' is zero (between 0 and 255)
#define LINE_FOLLOW_MOTOR_SWING             255 //amount to swing from 'MOTOR_SPEED' as 'correction' varies. probably should be as big as speed

// ### Distance Sensors ###
#define INVALID_READING     -1 //signifies an invalid distance reading
//IR constants
#define IR_AVERAGE_READINGS 4 //A=amount of readings averaged for the IR distance
#define IR_WAIT_TIME        500//100 //amount of time to wait for IR readings to catch up
//Ultrasound constants
#define ULTRASOUND_TIMEOUT  3000 //max time to wait for ultrasound to return

// ###### PIN ASIGNMENTS ######

//Analog pins
#define LINE_SENSOR_PIN         A0 //connection to subtraction circuit
#define IR_SENSOR_PIN           A1 
//Digital pins
#define RESET_BUTTON            2
#define JUNCTION_SENSOR_PIN     3
#define MAGNET_SENSOR_PIN       4
#define LEFT_BUMPER_PIN         5
#define RIGHT_BUMPER_PIN        6
#define LIGHT_SENSOR            7
#define AMBER_LED_PIN           8
#define SERVO_PIN               9
#define ULTRASOUND_TRIGGER_PIN  10
#define ULTRASOUND_ECHO_PIN     11
#define RED_LED                 12
#define GREEN_LED               13
// Motor shield motor numbers (1-4)
#define LEFT_MOTOR_NUM          1
#define RIGHT_MOTOR_NUM         2

// ###### STATE SYSTEM ENUMS ######

// ANY CHANGES TO THESE, please replicate in the top of pc_debug.py

enum Purpose{ //Describes the current goal
    EXIT_START_BOX          = 0,
    TRAVEL_TO_FAR_SIDE      = 1, 
    PICK_UP_BLOCK           = 2,
    TRAVEL_TO_START_SIDE    = 3,
    DROP_BLOCK              = 4,
    RETURN_HOME             = 5
};

enum Location{
    SQUARE                  = 0,
    DROPOFF_SIDE            = 1,
    RAMP                    = 2,
    COLLECTION_SIDE         = 3,
    CROSS                   = 4,
    BLOCK_COLLECTION_AREA   = 5,
    TUNNEL                  = 6,
    RED_SQUARE              = 7,
    GREEN_SQUARE            = 8
};

enum Task{  //Describes the exact task the robot is peforming
    STOPPED         = 0,
    MOVE_FORWARD    = 1,
    REVERSE         = 2,
    TURN_LEFT       = 3,
    TURN_RIGHT      = 4,
    FOLLOW_LINE     = 5,
    SLOW_SWEEP      = 6,
    TURN_AROUND     = 7,
    RECOVERY        = 8,   
    FINDING_BLOCK   = 9
};

// struct to store the robot's state.
struct{
    Purpose purpose;
    Location location;
    Task task;
    bool isLost; //Enables recovery mode
    bool wrongWay; //Indicates robot is heading the wrong direction along the line
    double task_timer; // acts as a countdown for the current action in miliseconds
    double task_stopwatch; // acts as a countup for the current action. useful to detect when an action is taking too long, so maybe the robot is lost
    int junction_counter; //counts the number of junctions passed
    bool is_holding_block=false;
    bool is_magnetic=false; //is the block being held magnetic
    bool return_home=false; //is the robot aiming to head home
    int circuit_count=0;    //how many laps has the robot completed
    //remember to add any new options to ResetState() as well
} RobotState; //name of struct