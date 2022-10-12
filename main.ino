/*
Main code file for the IDP robot
Target loop speed is set by 'tick_time'.
*/

//include header files
#include "src/include/LineSensor.h"
#include "src/include/WifiDebug.h"
#include "src/include/Motorcontrol.h"


//Create objects to access modules
LineSensor LineSense;
WifiDebug Debug;
Motorcontrol Mcon;


//timer global variables
int timer_last_value=0; //last time in microseconds
int tick_time=10;//target tick time in ms
int max_tick_time_exceeded=0;//biggest time by which the tick_time has been exceeded recently (in microseconds). This ought to be zero


void setup(){
    Serial.begin(57600); //setup serial
    LineSense.LineSensorSetup();
    Debug.SetupHotspot(); // Setup wifi debugging
    Mcon.MotorSetup();

    //setup timer
    timer_last_value=micros();
}


int m=0;//miliseconds counter (between 0 and 999), in increments of 'tick_time'
int s=0;//seconds counter

void loop(){ 
    // ### TIMER CODE ### 

    //Aims to delay for 'tick_time' and records if it takes any longer.
    //if the loop is taking longer it cannot adjust for this and so the clock will run slower, but the clock will never run fast.
    int dt=micros()-timer_last_value; // elapsed time in us
    if(dt<tick_time*1000){//if elapsed time is less than the desired delay, delay the remaining time.
        delayMicroseconds((tick_time*1000)-dt);
        dt=tick_time*1000;
    }else{//record that the tick time has been exceeded
        if(dt-tick_time*1000>max_tick_time_exceeded)
        max_tick_time_exceeded=dt-tick_time*1000;
    }
    timer_last_value=micros();
    //increment timer counters
    m+=tick_time;
    if(m>=1000){
        m=0;
        s++;
    }

    // ### REGULAR EVENTS ###

    // Note, all modulus events in miliseconds must be multiples of tick_time to trigger
    if(m%100==0){ // 10 times per second
        //print if the tick_time was exceeded
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
    LineSense.LineSensorUpdate(dt);



    // ### READ COMMANDS FROM PC ###
    String PC_reply=Debug.ReadCommand();
    if(PC_reply!=""){
        PC_Command(PC_reply);
    }
}

//called when the PC sends a command message. Debug.SendMessage sends a reply
void PC_Command(String command){
    // Graph plotting values
    if(command=="R"){ //Raw sensor input
        Debug.SendMessage(String(LineSense.differential_reading,0));
    }
    if(command=="I"){ // Integral
        Debug.SendMessage(String(LineSense.integral,3));
    }
    if(command=="D"){ // Derivitive
        Debug.SendMessage(String(LineSense.derivitive,3));
    }
    if(command=="E"){ // Error
        Debug.SendMessage(String(LineSense.error,3));
    }
    if(command=="C"){ // Correction
        Debug.SendMessage(String(LineSense.correction,3));
    }
}