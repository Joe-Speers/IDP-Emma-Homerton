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
    if(m%500==0){ // twice a second
        //print out the clock
        Serial.println("t= "+String(s)+":"+String(m));
    }

    // ### UPDATE SUBSYSTEMS ###
    // line following
    double correction = LineSense.PIDLineFollowCorrection(dt);
    if (s < 20){
        strt.startmovement(s, m, Mcon,Debug); //
    }
    else{
        if(s==20 and m==0) Debug.SendMessage("Following line");
        
        Mcon.MotorControlUpdate(correction);
    }

    // ### READ COMMANDS FROM PC ###
    String PC_reply=Debug.ReadCommand();
    if(PC_reply!=""){
        PC_Command(PC_reply);
    }
}

//called to reset the time and state to an inital value
void ResetState(){
    Debug.SendMessage("Resetting State");
    m=0;
    s=0;
    //reset states to inital values
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
