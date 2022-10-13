class LineSensor{
    public:
        float differential_reading=0;
        double correction=0;
        float integral=0; //stores the integral of 'error'
        float derivitive=0;
        double error=0;
        void LineSensorSetup(); //Setup call to initilise sensors
        double PIDLineFollowCorrection(int dt_micros); // call every loop to update program. dt_micros is the elapsed time. Returns correction value
};