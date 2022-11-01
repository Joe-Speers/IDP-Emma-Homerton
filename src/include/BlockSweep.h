#include "util.h"
#include "Motorcontrol.h"
#include "DistanceSense.h"
#include "LineSensor.h"

class BlockSweep
{
    public:
        enum SweepState{
            ROTATE_TO_OFFSET = 0,
            MOVE_OFFSET = 1,
            ROTATE_TO_SWEEP_START = 2,
            START_SWEEP = 3,
            ROTATE_TO_BLOCK = 4,
            WAIT_FOR_IRSENSOR = 5,
            MOVE_TO_ACCURATE_MEASURE_POINT =6,
            MOVE_TO_WITHIN_ULTRASOUND_RANGE = 7,
            PAUSE_BEFORE_FINAL_APPROACH  =8,
            MOVE_INTO_MAGNET = 9,
            DETECT_MAGNET = 10,
            LINE_NOT_FOUND = 11,
            GRAB_BLOCK = 12,
            REVERSE_TO_LINE = 13,
            ROTATE_FORWARD = 14,
            SWEEP_COMPLETE = 15,
            
        };
        int angle = 0;
        int distance = 0;
        

        SweepState BlockSwp(MotorControl &Mcon, DistanceSense &Dsense, WifiDebug &Debug);
        SweepState ReturnToCross(MotorControl &Mcon, DistanceSense &Dsense, LineSensor &Lsense, WifiDebug &Debug);
        //temp public
        SweepState laststate = ROTATE_TO_OFFSET; 
        unsigned long starttime = 0;
    private:
        
        
        unsigned long firstdetect;
        int overshoot = 0;
        int angleofblock = 700;
        int blockdistance = 0;
        
        bool blockdetected = 0;
        int crossangle = 0;
        int crossdistance = 0;

};