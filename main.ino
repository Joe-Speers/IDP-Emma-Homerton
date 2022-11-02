/*
main.ino
Main code for team 108's IDP robot.
The main file:
    - initilises modules and peforms startup operations
    - sets the initial state
    - contains an update loop and timing system to trigger time based events
    - handles commands recieved from a PC
    - contains a state system to handle changes in state and peform new actions. (todo)
Target interval for loop() is set by 'TICK_TIME' in miliseconds.
*/

//include header files

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

#define TICK_TIME 10 //target tick time in ms. Everything relies on this being 10ms

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
int max_tick_time_exceeded=0;//biggest time by which the TICK_TIME has been exceeded recently (in microseconds). This ought to be zero
int m=0;//miliseconds counter (between 0 and 999), in increments of 'TICK_TIME'
int s=-10000;//seconds counter

int error=0;

//called to reset the time and state to an inital value
void ResetState(){
    Debug.SendMessage("Resetting State");
    Mcon.SetMotors(0,0,FORWARD,FORWARD);
    Mcon.ResetState();
    Mcon.ResetMovement();
    TiltSense.reset();
    RobotState.junction_counter=0;
    m=0;
    //reset states to inital values
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
    //temp
    BSweep.laststate=BlockSweep::ROTATE_TO_OFFSET;
}

void setup(){
    Serial.begin(57600); //setup serial
    LineSense.LineSensorSetup();
    Mcon.MotorSetup();
    Debug.SetupHotspot(); // Setup wifi debugging
    distanceSense.SensorSetup();
    TiltSense.sensorSetup();
    TunnelSense.sensorSetup();
    magnetSense.sensorSetup();
    TiltSense.reset();
    //setup timer
    timer_last_value=micros();
    //set state variables
    ResetState();
    pinMode(AMBER_LED_PIN, OUTPUT);
    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(RESET_BUTTON,INPUT);
    
}
//temp for distance calibration
int move=0;
int turn=0;
void loop(){ 
    error+=1;
    if(error>1){
        Serial.println("MAIN LOOP CRITICAL ERROR: "+String(error));
        if(error==2){
            Debug.SendMessage("CRITICAL ERROR");
        }
    }
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
    //increment timer counters
    m+=TICK_TIME;
    if(m>=1000){
        m=0;
        s++;
    }

    // ### REGULAR EVENTS ###

    // Note, all modulus events in miliseconds must be multiples of TICK_TIME to trigger
    if(m%100==0){ // 10 times per second
        //print if the TICK_TIME was exceeded
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
        state_update_message+="!R"+String(RobotState.isLost)+"\n"; // recovery
        state_update_message+="!C"+String(RobotState.task_timer,1)+"\n"; //countdown
        state_update_message+="!S"+String(RobotState.task_stopwatch,1)+"\n"; //stopwatch
        state_update_message+="!J"+String(RobotState.junction_counter)+"\n";
        Debug.SendMessage(state_update_message);
    }
    if(m%500==0){ // twice a second
        //print out the clock
        //Debug.SendMessage("Distance: "+String(distanceSense.ReadUltrasoundDistance(),1)); has a long delay!
        //Debug.SendMessage("t: "+String(s)+":"+String(m));
        int temp=0;
    }
    if(!LineSense.LastJunctionDetectionState && LineSense.juntionDetect() && !RobotState.isLost){
        RobotState.junction_counter+=1;
    }
    LineSense.LastJunctionDetectionState=LineSense.juntionDetect();
    // ### UPDATE SUBSYSTEMS ###
    //temp calibration script
    if(false){
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
    } else if(false){//true if calibrating motors
        if(move>0){
            if(Mcon.MoveSetDistance(move)== COMPLETE){
                move=0;
            }
        }
        if(turn>0){
            if(Mcon.TurnSetAngle(turn,true)== COMPLETE){
                turn=0;
            }
        }
    } else {
        StateSystemUpdate(dt); // update state system
    }
    TiltSensor::TiltState tilt = TiltSense.getTilt(dt/1000); // update tilt sensor
    if(m%20==0){
        //Serial.println(String(TiltSense.y_average));
        Serial.println(String(distanceSense.ReadIRDistance()));
        Serial.println(","+String(distanceSense.ReadUltrasoundDistance()));
        //Serial.print(String(LineSense.derivative));
        //Serial.println(","+String(LineSense.error));
    }
    // peform PID calculation  
    double correction = LineSense.PIDLineFollowCorrection(dt);
    //amber LED code
    
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
    if((magnetSense.MagnetDetected() && !RobotState.is_holding_block) ||(RobotState.is_magnetic && RobotState.is_holding_block) ){
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GREEN_LED, LOW);
    } else{
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
    }
    if(digitalRead(RESET_BUTTON)==HIGH){
        if(s<0){
            s=0;
        }
        ResetState();
        digitalWrite(AMBER_LED_PIN, HIGH);
        
    }
    //if(LineSense.isLineDetected()){
    //     digitalWrite(AMBER_LED_PIN, HIGH);
    //} else {
    //   
    //    digitalWrite(AMBER_LED_PIN, LOW);
    //}
    //if following line, apply PID calculation
    if(RobotState.task==FOLLOW_LINE && !RobotState.isLost){
        bool linesense = LineSense.isLineDetected();
        bool followingLine=false;
        if(!linesense && (RobotState.location!=RAMP || TiltSense.getTilt()==TiltSensor::HORIZONTAL)){
            followingLine=Mcon.LineFollowUpdate(correction,false,Debug);
        } else {
            followingLine=Mcon.LineFollowUpdate(correction,true,Debug);
        }
        
        if(!followingLine){
            RobotState.isLost = true;
            Debug.SendMessage("Failed to find line, now lost");
        }
    }
    // ### Wifi Debug Read ###
    //read command from PC
    String PC_reply=Debug.ReadCommand();
    if(PC_reply!=""){
        PC_Command(PC_reply);
    }
    error=0;
}

void StateSystemUpdate(int elapsed_time_us){ //takes the elapsed time in microseconds as an input
    //increment task stopwatch
    RobotState.task_stopwatch+=elapsed_time_us/1000;
    //decrement task timer
    RobotState.task_timer-=elapsed_time_us/1000;
    //if timer is expected to complete within this tick then set to zero.
    if(RobotState.task_timer<TICK_TIME/2){
        RobotState.task_timer=0;
    }

    //If robot is lost then do something different
    if(RobotState.isLost){
        //recovery mode here
        //Mcon.SetMotors(0,0);//stop robot at the moment
        if (!RobotState.is_holding_block && (RobotState.location == COLLECTION_SIDE || RobotState.location == CROSS || RobotState.location == BLOCK_COLLECTION_AREA)){
            if (recovery.blockSite(Mcon, Debug, LineSense, distanceSense) == Recovery::LINE_FOUND){
                RobotState.isLost = false;
                Mcon.ResetMovement();
            }
            
        }else{
            Mcon.ResetState();
            Mcon.ResetMovement();
            RobotState.isLost=false;
        }
        return;// do not proceed to descision tree
    }

    // ### STATE SYSTEM DESCISION TREE
    // state system plan here: https://docs.google.com/spreadsheets/d/1c6zy2WIi2YzP9drrig8WuBd60yu9RLzDmgVKKxzxKIg/edit?usp=sharing
    //I'm trying to keep this in chronological order of actions.
    // Nested order: Purpose then Location then Task
    // check for single and double equal signs! the compiler does not seem to catch these errors
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
                //if(RobotState.task_stopwatch>10000) RobotState.isLost=true; //if ramp has not been hit after 10 seconds then the robot is lost
                //ignore any tilt readings untill enough time has passed. Also reset if tilting down for some reason
                if(distanceSense.ReadIRDistance()<35 && distanceSense.ReadIRDistance()!=INVALID_READING && m==0){
                    Debug.SendMessage("Near ramp");
                    Mcon.SetServoAngle(ARMS_CLOSED_ANGLE);
                    //RobotState.task_timer=3000; //add this if useful.
                    //add anything here?
                }
                if(RobotState.task_stopwatch<4000 && TiltSense.getTilt()==TiltSensor::TILT_DOWN){
                    TiltSense.reset();
                } else if(TiltSense.getTilt()==TiltSensor::TILT_UP || (RobotState.task_timer>0 && RobotState.task_timer<500 && false)){ // 4) check tilt sensor to see if has hit ramp
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
                if(ultrasoundDist<DISTANCE_MEASURE_MAGNET+4 && ultrasoundDist!=INVALID_READING &&RobotState.circuit_count==0 && RobotState.task_stopwatch>11000){
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
                            BSweep.laststate=BlockSweep::ROTATE_TO_OFFSET;
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
                if(TiltSense.getTilt()==TiltSensor::TILT_UP && false){ // 4) disabled for now
                    Debug.SendMessage("Unexpected Ramp hit");
                    RobotState.task = REVERSE;
                    Mcon.ResetMovement();
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
            if (RobotState.task == RECOVERY && false){ //disabled for now
                if(recovery.blockSite(Mcon, Debug, LineSense, distanceSense) == Recovery::LINE_FOUND){
                    LineSense.ResetPID();
                    RobotState.task = FOLLOW_LINE;
                    Mcon.ResetMovement();
                    Mcon.LineFollowUpdate(0,false,Debug,true);
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
                    Mcon.ResetMovement();
                    RobotState.purpose=PICK_UP_BLOCK;
                    RobotState.location=COLLECTION_SIDE;
                    RobotState.task=STOPPED;
                    Mcon.SetMotors(0,0);
                    RobotState.task_timer=100;
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
                    if((RobotState.junction_counter==1 && !RobotState.is_magnetic) || (RobotState.junction_counter>=3 && RobotState.is_magnetic)){ // temporary loop back to start
                        RobotState.purpose=DROP_BLOCK;
                        RobotState.task=MOVE_FORWARD;
                        Mcon.ResetMovement();
                        RobotState.task_stopwatch=0;
                        TiltSense.reset();
                    }
                } else {
                    if(RobotState.return_home){
                        Debug.SendMessage("aiming home");
                        RobotState.purpose=RETURN_HOME;
                        RobotState.location=DROPOFF_SIDE;
                        RobotState.task=MOVE_FORWARD;
                        Mcon.ResetMovement();
                        RobotState.task_stopwatch=0;
                        TiltSense.reset();
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
                if(Mcon.TurnSetAngle(100,false)==COMPLETE){
                    if(RobotState.return_home && !RobotState.is_magnetic){//check it is in first box
                        RobotState.junction_counter=0;
                        RobotState.task=FOLLOW_LINE;
                        RobotState.location=DROPOFF_SIDE;
                        RobotState.purpose=TRAVEL_TO_START_SIDE;
                        Mcon.ResetMovement();
                        Mcon.LineFollowUpdate(-1,true,Debug,true);
                        RobotState.task_stopwatch=0;
                    } else {
                        RobotState.task=FOLLOW_LINE;
                        RobotState.location=DROPOFF_SIDE;
                        RobotState.purpose=TRAVEL_TO_FAR_SIDE;
                        RobotState.task_stopwatch=0;
                        Mcon.ResetMovement();
                        Mcon.LineFollowUpdate(-1,true,Debug,true);
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
                if(Mcon.TurnSetAngle(90,true)==COMPLETE){
                    Mcon.SetServoAngle(ARMS_CLOSED_ANGLE);
                    RobotState.task=MOVE_FORWARD;
                    RobotState.location=START_SQUARE;
                    Mcon.ResetMovement();
                    RobotState.circuit_count+=1;
                } 
            }
        } else if (RobotState.location==START_SQUARE){
            if(RobotState.task==MOVE_FORWARD){
                if(Mcon.MoveSetDistance(32)==COMPLETE){
                    RobotState.task=STOPPED;
                    RobotState.task_stopwatch=0;
                    Mcon.ResetMovement();
                    Mcon.SetMotors(0,0);
                }
            }
        }
    }
}

//called when the PC sends a command message. Debug.SendMessage sends a reply
void PC_Command(String command){
    if(command=="RESET"){
        ResetState();
        s=0;
    }
    if(command=="STOP"){
        ResetState();
        RobotState.task=STOPPED;
        RobotState.location=START_SQUARE;
        RobotState.purpose=EXIT_START_BOX;
        s=-10000; //set timer to -10000 seconds to prevent state system from starting
    }
    if(command=="~H"){ //Raw sensor input
        RobotState.return_home=true;
        Debug.SendMessage("returning home");
    }
    if(command[0]=='M'){
        move=command.substring(1).toInt();
    }
    if(command[0]=='T'){
        turn=command.substring(1).toInt();
    }
    if(command[0]=='A'){
        if(command[1]=='P'){
            LineSense.proportional_k=command.substring(2).toFloat();
            Debug.SendMessage("Set proportional_k to "+String(LineSense.proportional_k,5));
        }
        if(command[1]=='I'){
            LineSense.integral_k=command.substring(2).toFloat();
            Debug.SendMessage("Set integral_k to "+String(LineSense.integral_k,5));
        }
        if(command[1]=='D'){
            LineSense.derivative_k=command.substring(2).toFloat();
            Debug.SendMessage("Set derivitive_k to "+String(LineSense.derivative_k,5));
        }
        if(command[1]=='L'){
            LineSense.integral_limit=(1/LineSense.integral_k)*command.substring(2).toFloat();
            Debug.SendMessage("Set integral_limit to "+String(LineSense.integral_limit,5));
        }
    }
    // Graph plotting values
    if(command=="R"){ //Raw sensor input
        Debug.SendMessage(String(LineSense.differential_reading,0));
    }
    if(command=="I"){ // Integral
        Debug.SendMessage(String(LineSense.integral,3));
    }
    if(command=="D"){ // derivative
        Debug.SendMessage(String(LineSense.derivative,3));
    }
    if(command=="E"){ // Error
        Debug.SendMessage(String(LineSense.error,3));
    }
    if(command=="C"){ // Correction
        Debug.SendMessage(String(LineSense.correction,3));
    }
}
