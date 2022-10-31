#include "util.h"
#include "Motorcontrol.h"
#include "DistanceSense.h"

class BlockSweep
{
    public:
        enum SweepState{
            RORATE_TO_OFFSET = 0,
            MOVE_OFFSET = 1,
            ROTATE_TO_SWEEP_START = 2,
            START_SWEEP = 3,
            ROTATE_TO_BLOCK = 4,
            WAIT_FOR_IRSENSOR = 5,
            MOVE_TO_BLOCK = 6,
            LINE_NOT_FOUND = 7,
            GRAB_BLOCK = 8,
            ROTATE_TO_CROSS = 9,
            MOVE_TO_CROSS = 10,
            ROTATE_FORWARD = 11,
            SWEEP_COMPLETE = 12,
        };

        int angle = 0;
        int distance = 0;
        
        //
        SweepState BlockSwp(MotorControl Mcon, DistanceSense Dsense);
        SweepState ReturnToCross(MotorControl Mcon, DistanceSense Dsense);

    private:
        int milli;
        int starttime = 0;
        int firstdetect;
        int midpoint = 0;
        int midp = 0;
        int angleofblock = 700;
        int blockdistance = 0;
        SweepState laststate = RORATE_TO_OFFSET;
        bool blockdetected = 0;
        int crossangle = 0;
        int crossdistance = 0;

};