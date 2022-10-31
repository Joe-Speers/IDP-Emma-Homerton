#include "include/BlockSweep.h"
#include "include/Motorcontrol.h"
#include"include\util.h"
#include "include/DistanceSense.h"


BlockSweep::SweepState BlockSweep::BlockSwp(MotorControl Mcon, DistanceSense Dsense){

    milli = millis();

    if (laststate == ROTATE_TO_OFFSET){
        //Turns robot to be perpendicular to cross
        if (Mcon.TurnSetAngle(90, CLOCKWISE) == COMPLETE){
            Mcon.ResetMovement();
            laststate = MOVE_OFFSET;
        }
    } else if (laststate == MOVE_OFFSET){
        //Moves robot back an offset to eliminate the error caused by block being too close for the IR sensor
        if (Mcon.MoveSetDistance(CROSS_OFFSET)== COMPLETE){
            laststate = ROTATE_TO_SWEEP_START;
            Mcon.ResetMovement();
        }
    } else if (laststate == ROTATE_TO_SWEEP_START){
        //Rotates robot to face sweep start location
        if (Mcon.TurnSetAngle(90, ANTI_CLOCKWISE)== COMPLETE){
            laststate = START_SWEEP;
            Mcon.ResetMovement();
            starttime = milli;
        }
    } else if (laststate == START_SWEEP){
        //Turns robot 180 degrees whilst recording if block is sensed
        if (Mcon.TurnSetAngle(180, ANTI_CLOCKWISE)== COMPLETE){
            angleofblock = 180 - Mcon.TimeToAngleCon(midp);
            Mcon.ResetMovement();
            laststate = ROTATE_TO_BLOCK;
        }
        if (blockdetected == 0){
        //will work if block is placed closer than any part of the ramp/ tunnel, if not a linear distance equation is need
            if (distance < MIN_WALL_DISTANCE){
                //records time at which block is first detected
                blockdetected = 1;
                firstdetect = milli;
            }
        }
        if (blockdetected == 1){
            if (distance > MIN_WALL_DISTANCE){
                    //records time at which block is no longer detected
                    blockdetected = 0;
                    midpoint = int ((milli + starttime)/2);

                } 
        }
    } else if (laststate == ROTATE_TO_BLOCK){
        //Turns robot to face midpoint of block and records distance to block
        if (Mcon.TurnSetAngle(180 - angleofblock, ANTI_CLOCKWISE)== COMPLETE){
            blockdistance = distance;
            blockdistance -= GAP_LEFT_TO_BLOCK;
            laststate = WAIT_FOR_IRSENSOR;
            Mcon.ResetMovement();
            starttime = milli;
        }
    } else if (laststate == WAIT_FOR_IRSENSOR){
        if (milli + IR_WAIT_TIME > milli){
            laststate = MOVE_TO_BLOCK;
            Mcon.ResetMovement();
        }
    } else if (laststate == MOVE_TO_BLOCK){
        //Moves robot forwards to a set distance from the block
        if (Mcon.MoveSetDistance(blockdistance - GAP_LEFT_TO_BLOCK)){
            laststate = GRAB_BLOCK;
            Mcon.ResetMovement();
            crossangle = int((180 * CROSS_OFFSET * asin(sin(((90-angleofblock)*3.141592)/180)/blockdistance))/ 3.141592);
            crossdistance = int((sin(((90-angleofblock)*3.141592)/180)*CROSS_OFFSET)/sin(((crossangle)*3.141592)/180)); 
        }
    } else if (laststate == GRAB_BLOCK){
        
    }
}

BlockSweep::SweepState BlockSweep::ReturnToCross(MotorControl Mcon, DistanceSense Dsense){

    milli = millis();

    if (laststate == GRAB_BLOCK){
        laststate = ROTATE_TO_CROSS;
        Mcon.ResetMovement();
    }
    if (laststate == ROTATE_TO_CROSS){
        //Turns robot around
        if (angleofblock > 90){
            if (Mcon.TurnSetAngle(180 - crossangle, ANTI_CLOCKWISE)== COMPLETE){
                laststate = MOVE_TO_CROSS;
                Mcon.ResetMovement();
            }
        }
        else {
            if (Mcon.TurnSetAngle(180 - crossangle, CLOCKWISE)== COMPLETE){
                laststate = MOVE_TO_CROSS;
                Mcon.ResetMovement();
            }
        }
    }
    if (laststate == MOVE_TO_CROSS){
        //Moves robot back to cross
        if (Mcon.MoveSetDistance(crossdistance)== COMPLETE){
            laststate = ROTATE_FORWARD;
            Mcon.ResetMovement();
        }
    }
    if (laststate == ROTATE_FORWARD){
        //Rotates robot to face allong the line
        if (angleofblock > 90){
            if (Mcon.TurnSetAngle(180 - crossangle - angleofblock, ANTI_CLOCKWISE)== COMPLETE){
                laststate = SWEEP_COMPLETE;
                Mcon.ResetMovement();
            }
        }
        else {
            if (Mcon.TurnSetAngle(180 + crossangle - angleofblock, ANTI_CLOCKWISE)== COMPLETE){
                laststate = SWEEP_COMPLETE;
                Mcon.ResetMovement();
            }
        }
    }
}
