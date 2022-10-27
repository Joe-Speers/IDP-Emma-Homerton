#include "util.h"
#include "Motorcontrol.h"

class BlockSweep
{
    public:
        enum SweepState{
            RORATE_TO_OFFSET = 0,
            MOVE_OFFSET = 1,
            START_SWEEP = 2,
            ROTATE_TO_BLOCK = 3,
            MOVE_TO_BLOCK = 4,
            LINE_NOT_FOUND = 5,
            GRAB_BLOCK = 6,
            ROTATE_TO_CROSS = 7,
            MOVE_TO_CROSS = 8,
            ROTATE_FORWARD = 9,
            SWEEP_COMPLETE = 10,
        };

        int angle = 0;
        int distance = 0;
        
        //
        SweepState BlockSwe(MotorControl Mcon, int distance);

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