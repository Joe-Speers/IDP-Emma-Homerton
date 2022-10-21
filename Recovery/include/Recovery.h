/*
blocksite recovery 
starting side recovery
ramp recovery
tunnel recovery
*/

class Recovery
{
    public:

        //location based recovery modules
        void Recovery:blocksite(MotorControl Mcon, WifiDebug Debug, location, purpose, seconds, junction, usdistance, irdistance);
        void Recovery:start(MotorControl Mcon, WifiDebug Debug, location, purpose, seconds, junction, usdistance, irdistance);
        void Recovery:ramp(MotorControl Mcon, WifiDebug Debug, location, purpose, seconds, junction, usdistance, irdistance);
        void Recovery:tunnel(MotorControl Mcon, WifiDebug Debug, location, purpose, seconds, junction, usdistance, irdistance);

};
