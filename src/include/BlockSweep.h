/*
BlockSweep.h
BlockSweep handles the collection of the 2nd block (and further blocks) by:
- Moving the robot into position slightly back from the cross
- Sweeping 180 degrees to look for the block
- rotating to be exactly in line with the centre of a block
- moving to within ACCURATE_MEASURING_DISTANCE of the block
- using ultrasound and infrared to precisely guide the robot to within 3cm but not touching the block
- delaying while magnet is detected.

Then once the block is picked up, ReturnToCross() handles returning to the line
*/
#include "util.h"
#include "Motorcontrol.h"
#include "DistanceSense.h"
#include "LineSensor.h"

#define BLOCK_SWEEP_TIMEOUT 30000 //maximum time for BlockSwp to run before giving up (in ms)

class BlockSweep
{
    public:
        enum SweepState{ //All States for block sweeping and returning to the line.
            ROTATE_TO_OFFSET = 0,
            MOVE_OFFSET = 1,
            ROTATE_TO_SWEEP_START = 2,
            START_SWEEP = 3,
            ROTATE_TO_BLOCK = 4,
            WAIT_FOR_IRSENSOR = 5,
            MOVE_TO_ACCURATE_MEASURE_POINT =6,
            MOVE_TOWARDS_MAGNET = 7,
            MOVE_INTO_MAGNET = 9,
            DETECT_MAGNET = 10,
            LINE_NOT_FOUND = 11,
            GRAB_BLOCK = 12,
            REVERSE_TO_LINE = 13,
            ROTATE_FORWARD = 14,
            SWEEP_COMPLETE = 15,
        };

        SweepState BlockSwp(MotorControl &Mcon, DistanceSense &Dsense, WifiDebug &Debug);//Should be called every update loop while active. Returns the current SweepState stage.
        SweepState ReturnToCross(MotorControl &Mcon, DistanceSense &Dsense, LineSensor &Lsense, WifiDebug &Debug);//Should be called every update loop while active. Returns the current SweepState stage.

        SweepState sweep_state = ROTATE_TO_OFFSET;  // Current SweepState value. To reset, set to ROTATE_TO_OFFSET  
    private:
        int timeout=0; //stores a counter in ms for how long BlockSweep has been running for
        unsigned long starttime = 0; //timer used throughout blockswp
        unsigned long firstdetect; //time when block first detected
        int angleofblock; //angle between centre of block and line
        int overshoot = 0;  //angle between centre of block and robot position
        int blockdistance = 0; //distance from robot to block
        bool blockdetected = false; //true if block has been detected while sweeping
        int crossangle = 0; //angle to rotate to return to cross
};