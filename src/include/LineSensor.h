/*
LineSensor.h
LineSensor interfaces with the line sensing subtraction interface circuit, as well as the third line sensor used to detect junctions.
PIDLineFollowCorrection() takes readings and peforms PID control, it should be called as frequently as possible to provide good control.
The PID readings as well as the raw sensor reading can be accessed through the public variables.
*/
#pragma once

#include "util.h"

#define DERIVITIVE_PREVIOUS_READINGS_TO_AVERAGE 10

class LineSensor{
    public:
        float differential_reading=0;   //Raw sensor reading between 0 and 1023
        double error=0;                 //Normalised sensor reading between -1 and 1
        double correction=0;            //turning rate to be applied to the motors (-1 to 1)
        float integral=0;               //stores the integral of 'error'
        float derivative=0;             //stores the derivative of 'error'
        bool LastJunctionDetectionState=false; //was the last junction reading true or false?
        //PID control constants (not actually constant, can be adjusted live)
        float proportional_k=DEFAULT_PROPORTIONAL_K;
        float integral_k=DEFAULT_INTEGRAL_K;
        float derivative_k=DEFAULT_DERIVATIVE_K;
        float integral_limit=(1/integral_k)*DEFAULT_INTEGRAL_CORRECTION_LIMIT;// hard limit on integral size

        void ResetPID();                //resets integral and derivitive values
        void LineSensorSetup();         //Setup call to initilise sensors
        double PIDLineFollowCorrection(int dt_micros); // takes a sensor reading and peforms PID calculation. Returns correction value between -1 and 1. dt_micros is the elapsed time since this was last called.
        bool juntionDetect();           //returns binary output, 1 for junction detected, otherwise 0
        bool isLineDetected();          //Returns if a line is detected based upon the noise present in the reading
        
    private:
        float error_array[DERIVITIVE_PREVIOUS_READINGS_TO_AVERAGE]; // stores the last few values of 'error', used to calculate derivative
        int LostLineCounter=0;          //counts up the number of loops where the line is not present

};