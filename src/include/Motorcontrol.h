class MotorCon{
    public
        
        int left_motor;
        int right_motor;

        //motor turning settings
        int MOTOR_SPEED = 200;//speed when 'correction' is zero (between 0 and 255)
        int MOTOR_SWING = 200;//amount to swing from 'MOTOR_SPEED' as 'correction' varies. probably should be as big as speed

        void MotorSetup(); //Setup call to initilise sensors
        void MotorUpdate(int lmotor, int rmotor); // call every loop to update program. dt_micros is the elapsed time
        void MotorControlUpdate(double correction)

};

