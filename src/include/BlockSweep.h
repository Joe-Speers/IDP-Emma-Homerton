#include "util.h"

class BlockSweep
{
    public:
        int angle = 0;
        int distance = 0;
        
        //
        bool BlockSwe(int distance);
        int AngleFind(int distance, int milliseconds);

    private:
        int milli;
        int starttime = 0;
        int firstdetect;
        int midpoint = 0;
        int midp = 0;
        int angleofblock = 700;
        int blockdistance = 0;
        int sweepstate = 0;
        bool blockdetected = 0;
        bool ismoving = 0;

};