/*
Recovery modules to relocate line 
blocksite recovery 
starting side recovery
ramp recovery
tunnel recovery
*/
#include "DistanceSense.h"
#include "MotorControl.h"
#include "WifiDebug.h"
//TODO: commenting
//This code is incomplete, we were not able to implent this for the competition
#include "util.h"
#include "LineSensor.h"

class Recovery
{
    public:
        enum RecoveryState{
            RECOVERY_SETUP = 0,
            WALL_FIND = 1,
            ROTATE_TO_WALL = 2,
            FIND_LINE_BACKWARDS = 3,
            FIND_LINE_FORWARDS = 4,
            LINE_FOUND = 5,
            LINE_NOT_FOUND = 6,
            DISTANCE_TOO_SMALL = 7,
        };
        //location based recovery modules
        RecoveryState blockSite(MotorControl &Mcon, WifiDebug &Debug, LineSensor &Lsense, DistanceSense &Dsense);

    private:
        int distance;
        int milli;
        bool junct_prev; //whether the junction was previously detected
        bool junct_cur; //whether the junction is currently detected
        int start_timer;
        int wall_detect_time;
        int wall_angle;
        int closest_distance;
        int start_distance;
        RecoveryState LastState = RECOVERY_SETUP;

};
