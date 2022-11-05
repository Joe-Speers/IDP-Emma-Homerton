/*
main.ino
Main code for team 108's IDP robot.
This file interfaces with the modules in the 'src' folder. All the key procedures are kept in the main file and the modules handle sensor readings and outputs.
/src/include/util.h contains key constants used throughout main.ino and modules.

The main file:
    - initilises modules and peforms startup operations (setup())
    - sets the initial state and resets modules (ResetState())
    - contains a timed update loop (loop()) which:
        - Handles time based events
        - Updates subsystems and sensors
        - controls line following mode
        - Updates LEDs
        - checks for messages from a PC over wifi
    - contains a state system to handle all robot descisions and peform new actions. (StateSystemUpdate())
    - responds to commands recieved from a PC (PC_Command())

Target interval for loop() is set by 'TICK_TIME' in miliseconds. This is followed exactly by means of a timer and delay system.
*/

#include "src/include/LineSensor.h"
#include "src/include/WifiDebug.h"
#include "src/include/MotorControl.h"
#include "src/include/util.h"
#include "src/include/DistanceSense.h"
#include "src/include/TiltSensor.h"
#include "src/include/TunnelSensor.h"
#include "src/include/Recovery.h"
#include "src/include/MagnetSensor.h"
#include "src/include/BlockSweep.h"

#define TICK_TIME 10 //target loop time in ms.

//Create objects to access modules
LineSensor LineSense;
WifiDebug Debug;
MotorControl Mcon;
DistanceSense distanceSense;
TiltSensor TiltSense;
TunnelSensor TunnelSense;
Recovery recovery;
MagnetSensor magnetSense;
BlockSweep BSweep;

//timer global variables
unsigned long timer_last_value=0; //last time in microseconds
int max_tick_time_exceeded=0;//biggest time by which the TICK_TIME has been exceeded recently (in microseconds). If not zero, a warning message is sent.
int m=0;//miliseconds counter (between 0 and 999), in increments of 'TICK_TIME'
int s=-100000;//seconds counter. The state system begins when s=0.

//Initial power on setup routine (Arduino called)
void setup(){
    Serial.begin(57600); //setup serial communication
    //setup modules
    LineSense.LineSensorSetup();
    Mcon.MotorSetup();
    distanceSense.SensorSetup();
    TiltSense.sensorSetup();
    TunnelSense.sensorSetup();
    magnetSense.sensorSetup();
    TiltSense.reset();
    Debug.SetupHotspot(); // Setup wifi debugging
    //setup timer
    timer_last_value=micros();
    //reset state variables
    ResetState();
    //setup main controlled pins
    pinMode(AMBER_LED_PIN, OUTPUT);
    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(RESET_BUTTON,INPUT);
}

//called to reset the time and state to initial values. Also resets key components. Does not reset 's' timer so the competition time can be maintained.
void ResetState(){
    Debug.SendMessage("Resetting State");
    //reset modules
    Mcon.SetMotors(0,0,FORWARD,FORWARD);
    Mcon.ResetState();
    Mcon.ResetMovement();
    TiltSense.reset();
    BSweep.sweep_state=BlockSweep::ROTATE_TO_OFFSET;
    //reset milisecond counter
    m=0;
    //reset state system to inital values
    RobotState.junction_counter=0;
    RobotState.return_home=false;
    RobotState.location=START_SQUARE;
    RobotState.purpose=EXIT_START_BOX;
    RobotState.task=STOPPED;
    RobotState.isLost=false;
    RobotState.wrongWay=false;
    RobotState.task_timer=1000;
    RobotState.task_stopwatch=0;
    RobotState.junction_counter=0;
    RobotState.circuit_count=0;
    RobotState.is_magnetic=false;
    RobotState.is_holding_block=false;
}

//Main loop (Arduino called). Timer ensures this runs exactly every TICK_TIME.
void loop(){ 
    // ### TIMER CODE ### 
    //Aims to delay for 'TICK_TIME' and records if it takes any longer.
    //if the loop is taking longer it cannot adjust for this and so the clock will run slower, but the clock will never run fast.
    int dt=micros()-timer_last_value; // elapsed time in us
    if(dt<TICK_TIME*1000){//if elapsed time is less than the desired delay, delay the remaining time.
        delayMicroseconds((TICK_TIME*1000)-dt);
        dt=TICK_TIME*1000;
    }else{//record that the tick time has been exceeded
        if(dt-TICK_TIME*1000>max_tick_time_exceeded)
        max_tick_time_exceeded=dt-TICK_TIME*1000;
    }
    timer_last_value=micros();
    //increment milisecond and second timer counters
    m+=TICK_TIME;
    if(m>=1000){
        m=0;
        s++;
    }

    // ### REGULAR TIMED EVENTS ###
    // Note, all modulus conditions in miliseconds must be multiples of TICK_TIME to trigger
    if(m%100==0){ // 10 times per second print if the TICK_TIME was exceeded (clock is running slow)
        if(max_tick_time_exceeded>0){
            Serial.println("loop time exceeded by: "+String(max_tick_time_exceeded)+"us");
            max_tick_time_exceeded=0;//reset flag
        }
    }
    if(m%300==0){// 3 times a second, send the robot's state information over WiFi debug
        String state_update_message="";
        state_update_message+="!L"+String(RobotState.location)+"\n";
        state_update_message+="!P"+String(RobotState.purpose)+"\n";
        state_update_message+="!T"+String(RobotState.task)+"\n";
        state_update_message+="!R"+String(RobotState.isLost)+"\n"; // is in reconvery mode
        state_update_message+="!C"+String(RobotState.task_timer,1)+"\n"; //countdown
        state_update_message+="!S"+String(RobotState.task_stopwatch,1)+"\n"; //stopwatch
        state_update_message+="!J"+String(RobotState.junction_counter)+"\n"; //number of junctions passed
        Debug.SendMessage(state_update_message);
    }
    if(m%500==0){ // twice a second print out the clock
        Debug.SendMessage("t: "+String(s)+":"+String(m));
    }
    if(m%20==0){ //50Hz send useful information for debugging over serial. These can be plotted using Arduino Serial plotter
        //Serial.println(String(TiltSense.y_average));
        //Serial.println(String(distanceSense.ReadIRDistance()));
        //Serial.println(","+String(distanceSense.ReadUltrasoundDistance()));
        //Serial.print(String(LineSense.derivative));
        //Serial.println(","+String(LineSense.error));
    }
    
    // ### UPDATE SUBSYSTEMS ###
    //detect a junction and increment junction counter
    if(!LineSense.LastJunctionDetectionState && LineSense.juntionDetect() && !RobotState.isLost){
        RobotState.junction_counter+=1;
    }
    LineSense.LastJunctionDetectionState=LineSense.juntionDetect();
    // update tilt sensor
    TiltSensor::TiltState tilt = TiltSense.getTilt(dt/1000); 
    // update state descision system
    StateSystemUpdate(dt); 

    // ### LINE FOLLOWING ###
    // peform PID calculation from line sensor subtraction circuit
    double correction = LineSense.PIDLineFollowCorrection(dt); //correction is in range -1 to 1 and signifies if the robot should move left or right

    //if following line and not lost, use the PID correction value to set the motors
    if(RobotState.task==FOLLOW_LINE && !RobotState.isLost){
        bool isLineDetected = LineSense.isLineDetected(); //check robot is still on the line
        //disable recovery if going up or down the ramp
        bool isRecoveryEnabled = !(RobotState.location!=RAMP || TiltSense.getTilt()==TiltSensor::HORIZONTAL);
        //enter recovery mode if line is lost
        bool enterRecoveryMode = !isLineDetected && isRecoveryEnabled;
        //set motors using correction value
        bool followingLine=Mcon.LineFollowUpdate(correction,enterRecoveryMode,Debug); //returns false if it cannot find the line
        if(!followingLine){//if line cannot be found, enter full recovery mode and send a debug message
            RobotState.isLost = true;
            Debug.SendMessage("Failed to find line, now lost");
        }
    }

    // ### LEDs and Start/Reset button ###
    //Flashing Amber LED code, flashes at 2Hz
    if(RobotState.task==STOPPED){
        digitalWrite(AMBER_LED_PIN, LOW);
    }
    else {
        if (m == 500 || m ==0){
            digitalWrite(AMBER_LED_PIN, HIGH);
        }
        if (m == 100 || m == 600){
            digitalWrite(AMBER_LED_PIN, LOW);
        }
    }
    // Red / Green LED code, for magnet detection
    //if holding block, keep the LED constant, otherwise LED will be linked directly to sensor.
    if((RobotState.is_magnetic && RobotState.is_holding_block) ||(magnetSense.MagnetDetected() && !RobotState.is_holding_block)){
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GREEN_LED, LOW);
    } else{
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
    }
    //Reset / Start button.
    if(digitalRead(RESET_BUTTON)==HIGH){
        if(s<0){
            s=0;//set seconds counter to zero
        }
        ResetState();//reset all modules and key variables
        digitalWrite(AMBER_LED_PIN, HIGH);// Light up Amber LED while resetting
    }
    
    // ### Wifi Debug Recieve Messages ###
    String PC_reply=Debug.ReadCommand();// read command from PC
    if(PC_reply!=""){
        PC_Command(PC_reply);// PC_Command handles incoming messages
    }
}

//State Systems handles descision making based on the Robots Location, Purpose, Task, time and sensor input. Run every loop
void StateSystemUpdate(int elapsed_time_us){ //takes the elapsed time in microseconds as an input
    /// ### STOPWATCH AND TIMER ###
    //increment task stopwatch
    RobotState.task_stopwatch+=elapsed_time_us/1000;
    //decrement task timer
    RobotState.task_timer-=elapsed_time_us/1000;
    //if timer is expected to complete within this tick then set to zero.
    if(RobotState.task_timer<TICK_TIME/2){
        RobotState.task_timer=0;
    }

    // ### RECOVERY MODE ###
    //While robot is lost then do not follow state system descision tree, instead procedures in the Recovery Module are followed
    if(RobotState.isLost){
        //check robot is not holding block, otherwise recovery module will not work due to sensors being covered. Also check
        //that robot is in a recoverable location.
        if (!RobotState.is_holding_block && (RobotState.location == COLLECTION_SIDE || RobotState.location == CROSS || RobotState.location == BLOCK_COLLECTION_AREA)){
            //follow procedures in recovery module
            if (recovery.blockSite(Mcon, Debug, LineSense, distanceSense) == Recovery::LINE_FOUND){
                //if rediscovered line, return to normal operation
                RobotState.isLost = false;
                Mcon.ResetMovement();
            }
            
        }else{ //if recovery module is not applicable in this current state, try to return to normal state system.
            Mcon.ResetState();
            Mcon.ResetMovement();
            RobotState.isLost=false;
        }
        return;// do not proceed to descision tree while in recovery mode
    }

    // ### STATE SYSTEM DESCISION TREE
    // This handles all descisions and keeps track of the robots state. RobotState variables are visible over wifi Debug.
    // State system plan here: https://docs.google.com/spreadsheets/d/1c6zy2WIi2YzP9drrig8WuBd60yu9RLzDmgVKKxzxKIg/edit?usp=sharing
    // This is in approximate chronological order.
    // Each nested 'if' statement is: Purpose, then Location, then Task, then any condition required to proceed to the next state
    if(RobotState.purpose==EXIT_START_BOX){
        if(RobotState.location==START_SQUARE){
            if(RobotState.task==STOPPED){ // 0) This is the initial state after a reset
                if(RobotState.task_timer==0 && s>=0){ // 1) Start moving after 1 second
                    Debug.SendMessage("Robot starting");
                    Mcon.SetServoAngle(ARMS_OPEN_ANGLE);
                    RobotState.task=MOVE_FORWARD;
                    RobotState.task_stopwatch=0;
                }
            } else if(RobotState.task==MOVE_FORWARD){
                if(Mcon.MoveSetDistance(20)==COMPLETE){
                    RobotState.junction_counter=0;                   
                    RobotState.location=DROPOFF_SIDE;
                    RobotState.task=MOVE_FORWARD;
                    Mcon.ResetMovement();
                    Mcon.SetMotors(255,255);
                    RobotState.task_stopwatch=0;
                }
            }
        } else if(RobotState.location==DROPOFF_SIDE){
            if(RobotState.task==MOVE_FORWARD){
                if(RobotState.junction_counter>0 ||RobotState.task_stopwatch>1200){
                    if(Mcon.MoveSetDistance(DISTANCE_TO_ROTATION_POINT)==COMPLETE){
                        RobotState.junction_counter=0;                   
                        RobotState.task_stopwatch=0;
                        RobotState.purpose=TRAVEL_TO_FAR_SIDE;
                        RobotState.task=TURN_RIGHT; // 2) Start turning onto line
                        Mcon.ResetMovement();
                    }
                }
            }
        }
    } else if (RobotState.purpose==TRAVEL_TO_FAR_SIDE){
        if(RobotState.location==DROPOFF_SIDE){
            if(RobotState.task==TURN_RIGHT){
                if(Mcon.TurnSetAngle(90,true)==COMPLETE){ // 3) start following the line
                    RobotState.task=FOLLOW_LINE;
                    Mcon.ResetMovement();
                    TiltSense.reset();
                    LineSense.ResetPID();
                    Mcon.LineFollowUpdate(1,false,Debug,true);
                    RobotState.task_stopwatch=0;
                    RobotState.task_timer=0;
                    RobotState.junction_counter=0;
                }
            } else if(RobotState.task==FOLLOW_LINE){
                if(distanceSense.ReadIRDistance()<35 && distanceSense.ReadIRDistance()!=INVALID_READING && m==0){
                    Debug.SendMessage("Near ramp");
                    Mcon.SetServoAngle(ARMS_CLOSED_ANGLE);
                }
                //ignore any tilt readings untill enough time has passed. Also reset if tilting down for some reason
                if(RobotState.task_stopwatch<4000 && TiltSense.getTilt()==TiltSensor::TILT_DOWN){
                    TiltSense.reset();
                } else if(TiltSense.getTilt()==TiltSensor::TILT_UP ){ // 4) check tilt sensor to see if has hit ramp
                    RobotState.location=RAMP;
                    RobotState.task_stopwatch=0;
                    RobotState.task_timer=0;
                }
            }
        } else if(RobotState.location==RAMP){      
            if(RobotState.task==FOLLOW_LINE){
                if(TiltSense.getTilt()==TiltSensor::TILT_UP){
                    if(TunnelSense.WallCollisionRight()){
                        LineSense.integral=LineSense.integral_limit*2;
                    }
                    else if(!LineSense.isLineDetected()){
                        LineSense.integral-=0.00005*(elapsed_time_us/1000); //make robot tend to go right to avoid falling off the ramp
                    }
                    if(RobotState.task_stopwatch>12000){
                        Debug.SendMessage("Stuck going up ramp");
                        //reverse back
                        Mcon.ResetMovement();
                        RobotState.task=REVERSE;
                    }
                }
                if(TiltSense.getTilt()==TiltSensor::TILT_DOWN){
                    RobotState.location=COLLECTION_SIDE;
                    Mcon.SetServoAngle(ARMS_OPEN_ANGLE);
                    RobotState.task=FOLLOW_LINE;
                    Mcon.ResetMovement();
                    RobotState.task_stopwatch=0;
                    RobotState.task_timer=0;
                }
                if((RobotState.task_timer==0 && TiltSense.getTilt()==TiltSensor::HORIZONTAL)&& RobotState.task_stopwatch>6500){
                    Debug.SendMessage("Must have completed ramp by now");
                    Mcon.SetServoAngle(ARMS_OPEN_ANGLE);
                    RobotState.location=COLLECTION_SIDE;
                    RobotState.task=FOLLOW_LINE;
                    RobotState.task_stopwatch=0;
                }
                if(TiltSense.getTilt()==TiltSensor::HORIZONTAL && RobotState.task_timer==0){
                    RobotState.task_timer=7000;
                    RobotState.task_stopwatch=0;
                }
            } else if(RobotState.task==REVERSE){
                if(Mcon.MoveSetDistance(-30)==COMPLETE){
                    RobotState.task=FOLLOW_LINE;
                    Mcon.ResetMovement();
                    Mcon.LineFollowUpdate(-0.0001,true,Debug,true);
                    RobotState.task_stopwatch=0;
                }
            }
        } else if(RobotState.location==COLLECTION_SIDE){
            if(RobotState.task==FOLLOW_LINE){
                float ultrasoundDist = distanceSense.ReadUltrasoundDistance(); 
                if(ultrasoundDist<ULTRASOUND_BLOCK_DETECTION_THRESHOLD && ultrasoundDist!=INVALID_READING &&RobotState.circuit_count==0 && RobotState.task_stopwatch>11000){
                    RobotState.purpose=PICK_UP_BLOCK;
                    RobotState.location=CROSS;
                    RobotState.task=MOVE_FORWARD;
                    Mcon.ResetMovement();
                    Mcon.MoveSetDistance(3);
                }
                if(LineSense.juntionDetect() && RobotState.task_stopwatch>11000){
                    if (RobotState.wrongWay==true){
                        Mcon.ResetMovement();
                        RobotState.task = MOVE_FORWARD;
                    }else {
                        if(RobotState.return_home){
                            Debug.SendMessage("skipping block pickup");
                            Mcon.SetServoAngle(ARMS_CLOSED_ANGLE);
                            RobotState.purpose=TRAVEL_TO_START_SIDE;
                            RobotState.location=COLLECTION_SIDE;
                            RobotState.task=FOLLOW_LINE;
                            Mcon.ResetMovement();
                            LineSense.ResetPID();
                            RobotState.task_stopwatch=0;
                        }
                        Debug.SendMessage("stopped at cross, time:" +String(RobotState.task_stopwatch));
                        if(RobotState.circuit_count==0){
                            RobotState.purpose=PICK_UP_BLOCK;
                            RobotState.location=CROSS;
                            RobotState.task=MOVE_FORWARD;
                            Mcon.ResetMovement();
                        } else {
                            RobotState.location==COLLECTION_SIDE;
                            RobotState.task=FINDING_BLOCK;
                            BSweep.sweep_state=BlockSweep::ROTATE_TO_OFFSET;
                            Mcon.ResetMovement();
                            Mcon.SetMotors(0,0);
                        }    
                    }
                }
                if(TunnelSense.TunnelDetected() && RobotState.task_stopwatch>5000){
                    Debug.SendMessage("Unexpected tunnel entry");
                    RobotState.location=COLLECTION_SIDE;
                    RobotState.task=REVERSE;
                    Mcon.ResetMovement();
                    RobotState.wrongWay = true;
                    RobotState.task_stopwatch=0;
                }
            }
            if (RobotState.task == REVERSE){
                if (Mcon.MoveSetDistance(-20) == COMPLETE){
                    Debug.SendMessage("turn around");
                    RobotState.task = TURN_AROUND;
                    Mcon.ResetMovement();
                }
            }
            if (RobotState.task == TURN_AROUND){
                if (Mcon.TurnSetAngle(180, ANTI_CLOCKWISE) == COMPLETE){
                    Debug.SendMessage("return to following line");
                    RobotState.task = FOLLOW_LINE;
                    Mcon.ResetMovement();
                    LineSense.ResetPID();
                    Mcon.LineFollowUpdate(0,false,Debug,true);
                }
            }
            if (RobotState.task == MOVE_FORWARD){
                if (Mcon.MoveSetDistance(30)==COMPLETE){
                    RobotState.task = TURN_AROUND;
                    Mcon.ResetMovement();
                    RobotState.wrongWay = false;
                }
            }
            if(RobotState.task==FINDING_BLOCK){
                BlockSweep::SweepState sweepState = BSweep.BlockSwp(Mcon,distanceSense,Debug);
                if(sweepState==BlockSweep::DETECT_MAGNET && !RobotState.is_holding_block){
                    if(magnetSense.MagnetDetected()){
                        Debug.SendMessage("magnetic!");
                        RobotState.is_magnetic=true;
                        RobotState.is_holding_block=true;
                    } else {
                        Debug.SendMessage("Not magnetic");
                        RobotState.is_magnetic=false;
                        RobotState.is_holding_block=true;
                    }
                }
                if(sweepState==BlockSweep::GRAB_BLOCK){
                    //grab block
                    Mcon.SetServoAngle(ARMS_CLOSED_ANGLE);
                    Debug.SendMessage("picking up block");
                    Mcon.ResetMovement();
                    RobotState.purpose=PICK_UP_BLOCK;
                    RobotState.location=COLLECTION_SIDE;
                    RobotState.task=STOPPED;
                    Mcon.SetMotors(0,0);
                    RobotState.task_timer=2000;
                }
                if(RobotState.return_home){
                    Mcon.SetServoAngle(ARMS_CLOSED_ANGLE);
                    Debug.SendMessage("aborting to return home");
                    RobotState.purpose=TRAVEL_TO_START_SIDE;
                    RobotState.location=COLLECTION_SIDE;
                    RobotState.task=FOLLOW_LINE;
                    Mcon.ResetMovement();
                    LineSense.ResetPID();
                    RobotState.task_stopwatch=0;
                }
            }
        }
    } else if(RobotState.purpose==PICK_UP_BLOCK){
        if(RobotState.location==CROSS){ //only called on round 1
            if(RobotState.task==MOVE_FORWARD){
                if(Mcon.MoveSetDistance(10)==COMPLETE){
                    Mcon.SetMotors(0,0);
                    //magnet sensing code
                    if(magnetSense.MagnetDetected()){
                        Debug.SendMessage("magnetic!");
                        RobotState.is_magnetic=true;
                        RobotState.is_holding_block=true;
                    } else {
                        Debug.SendMessage("Not magnetic");
                        RobotState.is_magnetic=false;
                        RobotState.is_holding_block=true;
                    }
                    RobotState.task=STOPPED;
                    RobotState.task_stopwatch=0;
                    Mcon.SetServoAngle(ARMS_CLOSED_ANGLE);
                    RobotState.task_timer=2000;// just stop for 2 seconds while capturing block
                    Debug.SendMessage("picking up block");
                    //initiate pickup block
                }
            } else if(RobotState.task==STOPPED){
                if(RobotState.task_timer==0){//finished picking up block
                    RobotState.purpose=TRAVEL_TO_START_SIDE;
                    RobotState.location=COLLECTION_SIDE;
                    RobotState.task=FOLLOW_LINE;
                    Mcon.ResetMovement();
                    LineSense.ResetPID();
                    RobotState.task_stopwatch=0;
                }
            }
        } else if(RobotState.location==COLLECTION_SIDE){
            if(RobotState.task_timer==0){//finished picking up block
                BlockSweep::SweepState sweepState = BSweep.ReturnToCross(Mcon,distanceSense,LineSense,Debug);
                if(sweepState==BlockSweep::SWEEP_COMPLETE){
                    RobotState.purpose=TRAVEL_TO_START_SIDE;
                    RobotState.location=COLLECTION_SIDE;
                    RobotState.task=FOLLOW_LINE;
                    Mcon.ResetMovement();
                    LineSense.ResetPID();
                    RobotState.task_stopwatch=0;
                }
            }
        }
    } else if(RobotState.purpose==TRAVEL_TO_START_SIDE){
        if(RobotState.location==COLLECTION_SIDE){
            if(RobotState.task==FOLLOW_LINE){
                if(TunnelSense.TunnelDetected() && RobotState.task_stopwatch>5000){
                    Debug.SendMessage("Entered Tunnel");
                    RobotState.location=TUNNEL;
                    RobotState.task=MOVE_FORWARD;
                    Mcon.ResetMovement();
                    Mcon.SetMotors(255,255);
                    RobotState.task_stopwatch=0;
                    LineSense.ResetPID();
                }
            }
        } else if(RobotState.location==TUNNEL){
            if(RobotState.task==MOVE_FORWARD){
                if(TunnelSense.WallCollisionLeft()){
                    Debug.SendMessage("Hit Left Wall");
                    RobotState.task=TURN_RIGHT;
                    RobotState.task_stopwatch=0;
                    Mcon.ResetMovement();
                    Mcon.TurnSetAngle(25, CLOCKWISE);
                } else if(TunnelSense.WallCollisionRight()){
                    Debug.SendMessage("Hit Right Wall");
                    RobotState.task=TURN_LEFT;
                    RobotState.task_stopwatch=0;
                    Mcon.ResetMovement();
                    Mcon.TurnSetAngle(25, ANTI_CLOCKWISE);
                }
            }
            if(!TunnelSense.TunnelDetected() && RobotState.task_stopwatch>600){ // checks 600ms has elapsed from last turning event
                Debug.SendMessage("Leaving tunnel");
                RobotState.location=DROPOFF_SIDE;
                RobotState.junction_counter=0;
                RobotState.task_stopwatch=0;
                RobotState.task=MOVE_FORWARD;
                Mcon.ResetMovement();
                Mcon.MoveSetDistance(10);
            }
            if (RobotState.task == TURN_RIGHT){
                if (Mcon.TurnSetAngle(20, CLOCKWISE)==COMPLETE){
                    RobotState.task=MOVE_FORWARD;
                    Mcon.ResetMovement();
                    Mcon.SetMotors(255,255);
                }
            }
            if (RobotState.task == TURN_LEFT){
                if (Mcon.TurnSetAngle(20, ANTI_CLOCKWISE)==COMPLETE){
                    RobotState.task=MOVE_FORWARD;
                    Mcon.ResetMovement();
                    Mcon.SetMotors(255,255);
                }
            }
        } else if(RobotState.location==DROPOFF_SIDE){
            if(RobotState.task==MOVE_FORWARD){
                if(Mcon.MoveSetDistance(10)==COMPLETE){
                    Debug.SendMessage("Trying to find line after tunnel");
                    Mcon.ResetMovement();
                    Mcon.LineFollowUpdate(-0.0001,true,Debug,true);
                    RobotState.task=FOLLOW_LINE;
                    RobotState.task_stopwatch=0;
                    LineSense.ResetPID();
                }
            } else if(RobotState.task==FOLLOW_LINE){
                if(RobotState.task_stopwatch<2000){ //reset junction counter
                    RobotState.junction_counter=0;
                }
                if(RobotState.is_holding_block){
                    if((RobotState.junction_counter==1 && !RobotState.is_magnetic) || (RobotState.junction_counter>=3 && RobotState.is_magnetic)){
                        RobotState.purpose=DROP_BLOCK;
                        RobotState.task=MOVE_FORWARD;
                        Mcon.ResetMovement();
                        RobotState.task_stopwatch=0;
                        TiltSense.reset();
                    }
                } else {
                    if(RobotState.return_home){
                        if(RobotState.junction_counter==2){
                            Debug.SendMessage("aiming home");
                            RobotState.purpose=RETURN_HOME;
                            RobotState.location=DROPOFF_SIDE;
                            RobotState.task=MOVE_FORWARD;
                            Mcon.ResetMovement();
                            RobotState.task_stopwatch=0;
                        }
                    } else {//contine back round to get another block
                        Debug.SendMessage("no block to drop off");
                        RobotState.task=FOLLOW_LINE;
                        Mcon.ResetMovement();
                        RobotState.location=DROPOFF_SIDE;
                        RobotState.purpose=TRAVEL_TO_FAR_SIDE;
                        RobotState.task_stopwatch=0;
                    }
                }
            }
        }
    } else if(RobotState.purpose==DROP_BLOCK){
        if(RobotState.location==DROPOFF_SIDE){
            if(RobotState.task==MOVE_FORWARD){
                if(Mcon.MoveSetDistance(DISTANCE_TO_ROTATION_POINT)==COMPLETE){
                    RobotState.task=TURN_RIGHT;
                    Mcon.ResetMovement();
                }
            } else if(RobotState.task==TURN_RIGHT){
                if(RobotState.junction_counter==1){
                    if(Mcon.TurnSetAngle(90,true)==COMPLETE){
                        RobotState.task=MOVE_FORWARD;
                        RobotState.location=START_SQUARE;
                        Mcon.SetServoAngle(ARMS_OPEN_ANGLE);
                        Mcon.ResetMovement();
                        RobotState.circuit_count+=1;
                    }
                } else {
                    if(Mcon.TurnSetAngle(90,true)==COMPLETE){
                        RobotState.task=MOVE_FORWARD;
                        RobotState.location=START_SQUARE;
                        Mcon.SetServoAngle(ARMS_OPEN_ANGLE);
                        Mcon.ResetMovement();
                        RobotState.circuit_count+=1;
                        RobotState.is_holding_block=false;
                    }
                }
            }
        } else if (RobotState.location==START_SQUARE){
            if(RobotState.task==MOVE_FORWARD){
                if(Mcon.MoveSetDistance(15)==COMPLETE){
                    RobotState.task=STOPPED;
                    Mcon.SetServoAngle(ARMS_OPEN_ANGLE);
                    RobotState.task_timer=1000;
                    RobotState.task=REVERSE;   
                    Mcon.ResetMovement();
                }
            }else if(RobotState.task==STOPPED){
                if (RobotState.task_timer == 0){
                    Mcon.SetServoAngle(ARMS_OPEN_ANGLE);
                    RobotState.is_holding_block=false;
                    RobotState.task=REVERSE;
                    Mcon.ResetMovement();
                    RobotState.task_stopwatch=0;
                }
            }else if(RobotState.task==REVERSE){
                if(Mcon.MoveSetDistance(-20)==COMPLETE){
                    RobotState.is_holding_block=false;
                    RobotState.task=TURN_LEFT;
                    RobotState.task_stopwatch=0;
                    Mcon.ResetMovement();
                }
            }else if(RobotState.task==TURN_LEFT){
                if(Mcon.TurnSetAngle(110,false)==COMPLETE){
                    if(RobotState.return_home && !RobotState.is_magnetic){//check it is in first box
                        RobotState.junction_counter=1;
                        RobotState.is_holding_block=false;
                        RobotState.task=FOLLOW_LINE;
                        RobotState.location=DROPOFF_SIDE;
                        RobotState.purpose=TRAVEL_TO_START_SIDE;
                        Mcon.ResetMovement();
                        Mcon.LineFollowUpdate(1,true,Debug,true);
                        RobotState.task_stopwatch=0;
                    } else {
                        RobotState.task=FOLLOW_LINE;
                        RobotState.location=DROPOFF_SIDE;
                        RobotState.purpose=TRAVEL_TO_FAR_SIDE;
                        RobotState.task_stopwatch=0;
                        Mcon.ResetMovement();
                        Mcon.LineFollowUpdate(1,true,Debug,true);
                    }
                }
            }
        }
    } else if(RobotState.purpose==RETURN_HOME){
        if(RobotState.location==DROPOFF_SIDE){
            if(RobotState.task==MOVE_FORWARD){
                if(Mcon.MoveSetDistance(DISTANCE_TO_ROTATION_POINT)==COMPLETE){
                    Mcon.ResetMovement();
                    RobotState.task=TURN_RIGHT;
                }
            } else if(RobotState.task==TURN_RIGHT){
                if(Mcon.TurnSetAngle(100,true)==COMPLETE){
                    Mcon.SetServoAngle(ARMS_CLOSED_ANGLE);
                    RobotState.task=MOVE_FORWARD;
                    RobotState.location=START_SQUARE;
                    Mcon.ResetMovement();
                    RobotState.circuit_count+=1;
                } 
            }
        } else if (RobotState.location==START_SQUARE){
            if(RobotState.task==MOVE_FORWARD){
                if(Mcon.MoveSetDistance(31)==COMPLETE){
                    RobotState.task=STOPPED;
                    RobotState.task_stopwatch=0;
                    Mcon.ResetMovement();
                    Mcon.SetMotors(0,0);
                }
            }
        }
    }
}

//called when the PC sends a message over Wifi. Debug.SendMessage() sends a reply
void PC_Command(String command){
    if(command=="RESET"){//Reset Robot state back to start square
        ResetState();
        s=0;
    }
    if(command=="STOP"){//Reset state and set timer such that the robot will not start
        ResetState();
        s=-100000; //set timer to -100000 seconds to prevent state system from starting
    }
    if(command=="~H"){ //Instruct robot to return back to the start square
        RobotState.return_home=true;
        Debug.SendMessage("returning home");
    }
    if(command[0]=='A'){//Adjust PID line following constants live
        if(command[1]=='P'){// adjust proportional constant
            LineSense.proportional_k=command.substring(2).toFloat();
            Debug.SendMessage("Set proportional_k to "+String(LineSense.proportional_k,5));
        }
        if(command[1]=='I'){// adjust integral constant
            LineSense.integral_k=command.substring(2).toFloat();
            Debug.SendMessage("Set integral_k to "+String(LineSense.integral_k,5));
        }
        if(command[1]=='D'){// adjust derivitive constant
            LineSense.derivative_k=command.substring(2).toFloat();
            Debug.SendMessage("Set derivitive_k to "+String(LineSense.derivative_k,5));
        }
        if(command[1]=='L'){// adjust integral limit (input is the absolute limit)
            LineSense.integral_limit=(1/LineSense.integral_k)*command.substring(2).toFloat();//internal integral limit depends on current integral constant
            Debug.SendMessage("Set integral_limit to "+String(LineSense.integral_limit,5));
        }
    }
    // Send data from the line sensors
    if(command=="R"){ //Raw sensor reading from subtraction circuit
        Debug.SendMessage(String(LineSense.differential_reading,0));
    }
    if(command=="E"){ // Error (normalised sensor reading between -1 and 1)
        Debug.SendMessage(String(LineSense.error,3));
    }
    if(command=="I"){ // Integral of error
        Debug.SendMessage(String(LineSense.integral,3));
    }
    if(command=="D"){ // Derivative of error
        Debug.SendMessage(String(LineSense.derivative,3));
    }
    if(command=="C"){ // Correction output between -1 and 1 to send to motors
        Debug.SendMessage(String(LineSense.correction,3));
    }
}