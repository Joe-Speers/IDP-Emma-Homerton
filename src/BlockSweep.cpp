#include "include/BlockSweep.h"
#include "include/Motorcontrol.h"
#include"include\util.h"

BlockSweep::SweepState BlockSweep::BlockSwe(MotorControl Mcon, int distance){

    milli = millis();

    if (laststate == RORATE_TO_OFFSET){
        if (!Mcon.TurnSetAngle(90, CLOCKWISE)){
            laststate == MOVE_OFFSET;
        }
    }
    if (laststate == MOVE_OFFSET){
        if (!Mcon.MoveSetDistance(CROSS_OFFSET)){
            laststate == START_SWEEP;
            starttime = milli;
        }
    }

    if (laststate == START_SWEEP){
        //Turns robot 180 degrees whilst recording if block is sensed
        if (!Mcon.TurnSetAngle(180, ANTI_CLOCKWISE)){
            angleofblock = 180 - Mcon.TimeToAngleCon(midp);
            laststate == ROTATE_TO_BLOCK;
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
    if (laststate == ROTATE_TO_BLOCK){
        //Turns robot to face midpoint of block and records distance to block
        if (!Mcon.TurnSetAngle(180 - angleofblock, ANTI_CLOCKWISE)){
            blockdistance = distance;
            laststate = MOVE_TO_BLOCK;
        }
    }
    if (laststate == MOVE_TO_BLOCK){
        //Moves robot forwards to a set distance from the block
        if (Mcon.MoveSetDistance(blockdistance - GAP_LEFT_TO_BLOCK)){
            laststate == GRAB_BLOCK;
        }
    }
    if (laststate == GRAB_BLOCK){
        Mcon.SetServoAngle(ARMS_CLOSED_ANGLE);

        //add cross calculations here
        crossangle = 0;
        crossdistance = 0;
        //may need to add delay time if 10us is not enough
        laststate = ROTATE_TO_CROSS;
    }
    if (laststate == ROTATE_TO_CROSS){
        //Turns robot around
        if (angleofblock > 90){
            if (!Mcon.TurnSetAngle(180 + crossangle, ANTI_CLOCKWISE)){
                laststate = MOVE_TO_CROSS;
            }
        }
        else {
            if (!Mcon.TurnSetAngle(180 + crossangle, CLOCKWISE)){
                laststate = MOVE_TO_CROSS;
            }
        }
    }
    if (laststate == MOVE_TO_CROSS){
        //Moves robot back to cross
        if (!Mcon.MoveSetDistance(crossdistance)){
            laststate = ROTATE_FORWARD;
        }
    }
    if (laststate == ROTATE_FORWARD){
        //Rotates robot to face allong the line
        if (angleofblock > 90){
            if (!Mcon.TurnSetAngle(180 + crossangle - angleofblock, ANTI_CLOCKWISE)){
                laststate = SWEEP_COMPLETE;
            }
        }
        else {
            if (!Mcon.TurnSetAngle(180 + crossangle - angleofblock, CLOCKWISE)){
                laststate = SWEEP_COMPLETE;
            }
        }
    }
}
