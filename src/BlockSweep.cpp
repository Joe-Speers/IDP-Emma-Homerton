#include "include/BlockSweep.h"
#include "include/Motorcontrol.h"
#include "include/DistanceSense.h"
#include"include\util.h"

MotorControl MotorC;

BlockSweep::SweepState BlockSweep::BlockSwe(int distance){

    milli = millis();

    if (laststate == RORATE_TO_OFFSET){
        if (!MotorC.TurnSetAngle(90, CLOCKWISE)){
            laststate == MOVE_OFFSET;
        }
    }
    if (laststate == MOVE_OFFSET){
        if (!MotorC.MoveSetDistance(CROSS_OFFSET)){
            laststate == START_SWEEP;
            starttime = milli;
        }
    }

    if (sweepstate == 1){
        //Turns robot 180 degrees whilst recording if block is sensed
        if (!MotorC.TurnSetAngle(180, ANTI_CLOCKWISE)){
            laststate == TURN_TO_BLOCK;
        }
        if (blockdetected == 0){
        //will work if block is placed closer than any part of the ramp/ tunnel, if not a linear distance equation is need
            if (distance < MIN_WALL_DISTANCE){
                blockdetected = 1;
                firstdetect = milli;
            }
        }
        if (blockdetected == 1){
            if (distance > MIN_WALL_DISTANCE){
                    blockdetected = 0;
                    midpoint = int ((milli + starttime)/2);

                } 
        }
    }
    if (sweepstate == 2){
        //Turns robot to face midpoint of block and records distance to block
        angleofblock = 180 - MotorC.TimeToAngleCon(midp);
        ismoving = MotorC.TurnSetAngle(angleofblock, CLOCKWISE);
        if (ismoving == 0){
            sweepstate += 1;
            blockdistance = distance;
        }
    }
    if (sweepstate == 3){
        //Moves robot forwards to a set distance from the block
        ismoving = MotorC.MoveSetDistance(blockdistance - GAP_LEFT_TO_BLOCK);
        if (ismoving == 0){
            sweepstate += 1;
        }
    }
    if (sweepstate == 4){
        MotorC.SetServoAngle(ARMS_CLOSED_ANGLE);
        //may need to add delay time if 10us is not enough
        sweepstate += 1;
    }
    if (sweepstate == 5){
        //Turns robot around
        ismoving = MotorC.TurnSetAngle(180, ANTI_CLOCKWISE);
        if (ismoving == 0){
            sweepstate += 1;
        }
    }
    if (sweepstate == 6){
        //Moves robot back to cross
        ismoving = MotorC.MoveSetDistance(blockdistance - GAP_LEFT_TO_BLOCK);
        if (ismoving == 0){
            sweepstate += 1;
        }
    }
    if (sweepstate == 7){
        //Rotates robot to face allong the line
        ismoving = MotorC.TurnSetAngle(angleofblock, ANTI_CLOCKWISE);
        if (ismoving == 0){
            sweepstate += 1;
        }
    }
}
        
        
int BlockSweep::AngleFind(int distance, int milliseconds){
//function to check distance sensors and log start and end points of block, then return midpoint
    if (blockdetected == 0){
        //will work if block is placed closer than any part of the ramp/ tunnel, if not a linear distance equation is need
        if (distance < MIN_WALL_DISTANCE){
            blockdetected = 1;
            firstdetect = milliseconds;
        }
    }
    if (blockdetected == 1){
       if (distance > MIN_WALL_DISTANCE){
            blockdetected = 0;
            midpoint = int ((milliseconds + starttime)/2);

        } 
    }
    
    return midpoint;

}