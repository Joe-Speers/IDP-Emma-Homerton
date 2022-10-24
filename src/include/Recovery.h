/*
blocksite recovery 
starting side recovery
ramp recovery
tunnel recovery
*/
#include "MotorControl.h"
#include "WifiDebug.h"
#include "util.h"

class Recovery
{
    public:
        enum RecoveryState{
            LOCATION_FIND = 0,
            CONVERSIONS = 1,
            ROTATE_TO_LINE = 2,
            MOVE_TO_LINE = 3,
            ROTATE_FORWARD = 4,
        };
        //location based recovery modules
        RecoveryState blocksite(MotorControl Mcon, WifiDebug Debug, Location location, Purpose purpose, int usdistance);
        RecoveryState start(MotorControl Mcon, WifiDebug Debug, Location location, Purpose purpose, int usdistance);
        RecoveryState ramp(MotorControl Mcon, WifiDebug Debug, Location location, Purpose purpose, int usdistance);
        RecoveryState tunnel(MotorControl Mcon, WifiDebug Debug, Location location, Purpose purpose, int usdistance);

    private:
        int milli;
        int start_timer;
        RecoveryState LastState = LOCATION_FIND;

};
