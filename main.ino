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
#include "src/include/start.h"
#include "src/include/util.h"

#define TICK_TIME 10 //target tick time in ms. Ideally <10ms

//Create objects to access modules
LineSensor LineSense;
WifiDebug Debug;
MotorControl Mcon;
start strt;

//timer global variables
int timer_last_value=0; //last time in microseconds
int max_tick_time_exceeded=0;//biggest time by which the TICK_TIME has been exceeded recently (in microseconds). This ought to be zero
int m=0;//miliseconds counter (between 0 and 999), in increments of 'TICK_TIME'
int s=0;//seconds counter

// struct to store the robot's state.
struct{
    Purpose purpose;
    Location location;
    Task task;
    bool isLost;
    double task_timer; // acts as a countdown for the current action in miliseconds
    double task_stopwatch; // acts as a countup for the current action. useful to detect when an action is taking too long, so maybe the robot is lost
    int junction_counter; //counts the number of junctions passed
    //suggestions:
    //int blocks_collected
    //bool is_holding_block
    //bool is_magnetic
    //remember to add any new options to ResetState() as well

} RobotState; //name of struct

//called to reset the time and state to an inital value
void ResetState(){
    Debug.SendMessage("Resetting State");
    m=0;
    s=0;
    //reset states to inital values
    RobotState.location=START_SQUARE;
    RobotState.purpose=EXIT_START_BOX;
    RobotState.task=STOPPED;
    RobotState.isLost=false;
    RobotState.task_timer=0;
    RobotState.task_stopwatch=0;
    RobotState.junction_counter=0;
}

void setup(){
    Serial.begin(57600); //setup serial
    LineSense.LineSensorSetup();
    Debug.SetupHotspot(); // Setup wifi debugging
    Mcon.MotorSetup();

    //setup timer
    timer_last_value=micros();
    //set state variables
    ResetState();
}

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
        state_update_message+="SL"+String(RobotState.location)+"\n";
        state_update_message+="SP"+String(RobotState.purpose)+"\n";
        state_update_message+="ST"+String(RobotState.task)+"\n";
        state_update_message+="SR"+String(RobotState.isLost)+"\n"; // recovery
        state_update_message+="SC"+String(RobotState.task_timer,1)+"\n"; //countdown
        state_update_message+="SS"+String(RobotState.task_stopwatch,1)+"\n"; //stopwatch
        state_update_message+="SJ:"+String(RobotState.junction_counter)+"\n";
        Debug.SendMessage(state_update_message);
    }
    if(m%500==0){ // twice a second
        //print out the clock
        Debug.SendMessage("t: "+String(s)+":"+String(m));
    }

    // ### UPDATE SUBSYSTEMS ###
    StateSystemUpdate(dt); // update state system

    // peform PID calculation
    double correction = LineSense.PIDLineFollowCorrection(dt);
    //if following line, apply PID calculation
    if(RobotState.task==FOLLOW_LINE){
        Mcon.MotorControlUpdate(correction);
    }

    // ### Wifi Debug Read ###
    //read command from PC
    String PC_reply=Debug.ReadCommand();
    if(PC_reply!=""){
        PC_Command(PC_reply);
    }
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
        return;// do not proceed to descision tree
    }

    // ### STATE SYSTEM DESCISION TREE

    //I'm trying to keep this in chronological order of actions.
    // Nested order: Purpose then Location then Task
    // check for single and double equal signs! the compiler does not seem to catch these errors
    if(RobotState.purpose==EXIT_START_BOX){
        if(RobotState.location==EXIT_START_BOX){
            if(RobotState.task==STOPPED){ // 0) This is the initial state after a reset
                if(s>=5){ // 1) Start moving after 5 seconds
                    Debug.SendMessage("Robot starting");
                    RobotState.task=MOVE_FORWARD;
                    RobotState.task_stopwatch=0;
                    //need to implement function to replace the next two lines with a distance to travel.
                    Mcon.SetMotors(255,255);
                    RobotState.task_timer=2000; //move forward for 2000 seconds.
                }
            } else if(RobotState.task==MOVE_FORWARD){
                if(RobotState.task_timer==0){ //replace with junction detection test (will also need to add another step to move forward more before turning)
                    RobotState.task=TURN_RIGHT; // 2) Start turning onto line
                    RobotState.location=DROPOFF_SIDE;
                    RobotState.purpose=TRAVEL_TO_FAR_SIDE;
                    RobotState.task_stopwatch=0;
                    //need to implement function to replace the next two lines with an angle to turn.
                    Mcon.SetMotors(255,0);
                    RobotState.task_timer=1500;
                }
            }
        } 
    } else if (RobotState.purpose==TRAVEL_TO_FAR_SIDE){
        if(RobotState.location==DROPOFF_SIDE){
            if(RobotState.task==TURN_RIGHT){
                if(RobotState.task_timer==0){ // 3) start following the line
                    RobotState.task=FOLLOW_LINE;
                    RobotState.task_stopwatch=0;
                }
            } else if(RobotState.task==FOLLOW_LINE){
                if(RobotState.task_stopwatch>10000) RobotState.isLost=true; //if ramp has not been hit after 10 seconds then the robot is lost
                if(false){ // 4) check tilt sensor to see if has hit ramp (TODO)
                    RobotState.location=RAMP;
                    RobotState.task_stopwatch=0;
                }
            }
        }
    }
    
    //if (s < 20){
    //    strt.startmovement(s, m, Mcon,Debug);
    //}
    //else{
    //    if(s==20 and m==0) Debug.SendMessage("Following line");
    //    RobotState.task=FOLLOW_LINE;
    //}
}

//called when the PC sends a command message. Debug.SendMessage sends a reply
void PC_Command(String command){
    if(command=="RESET"){
        ResetState();
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
