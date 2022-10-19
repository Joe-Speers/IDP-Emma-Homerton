#include "include/BlockSweep.h"
#include "IDP-Emma-Homerton/src/include/Motorcontrol.h"
#include "IDP-Emma-Homerton/src/include/DistanceSense.h"
#include"IDP-Emma-Homerton\src\include\util.h"

MotorControl Mcon;

bool BlockSweep::BlockSwe(int s, int m, int distance){

    milli = s + (1000*m);
    if (sweepstate == 0){
        starttime = milli;
        sweepstate += 1;
    }
    if (sweepstate == 1){
        ismoving = Mcon.TurnSetAngle(180, s, m, Anticlockwise);
        midp = AngleFind();
        if (ismoving == 0){
            sweepstate += 1;
        }
    }
    if (sweepstate == 2){
        angleofblock = 180 - Mcon.TimeToAngleCon(midp);
    }
    if (sweepstate == 3){
        ismoving = Mcon.TurnSetAngle(angleofblock, s, m, clockwise);
        if (ismoving == 0){
            sweepstate += 1;
            blockdistance = distance;
        }
    }
    if (sweepstate == 3){
        ismoving = Mcon.MoveSetDistance(blockdistance - GapLeftToBlock, s, m);
        if (ismoving == 0){
            sweepstate += 1;
        }
    }
    if (sweepstate == 4){
        //add block picking up here
        sweepstate += 1
    }
    if (sweepstate == 5){
        ismoving = Mcon.TurnSetAngle(180, s, m, Anticlockwise);
        if (ismoving == 0){
            sweepstate += 1;
        }
    }
    if (sweepstate == 6){
        ismoving = Mcon.MoveSetDistance(blockdistance - GapLeftToBlock, s, m);
        if (ismoving == 0){
            sweepstate += 1;
        }
    }
    if (sweepstate == 7){
        ismoving = Mcon.TurnSetAngle(angleofblock, s, m, anticlockwise);
        if (ismoving == 0){
            sweepstate += 1;
        }
    }
}
        
        
int BlockSweep::AngleFind(int distance, int milliseconds){
//function to check distance sensors and log start and end points of block, then return midpoint
    if (blockdetected == 0){
        //will work if block is placed closer than any part of the ramp/ tunnel, if not a linear distance equation is need
        if (distance < MinWallDistance){
            blockdetected = 1;
            firstdetect = milliseconds;
        }
    }
    if (blockdetected == 1){
       if (distance > MinWallDistance){
            blockdetected = 0;
            midpoint = int ((milliseconds + starttime)/2);

        } 
    }
    
    return midpoint;

}