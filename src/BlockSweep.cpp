/*
BlockSweep.cpp
See header file for description
*/
#include "include/BlockSweep.h"
#include "include/Motorcontrol.h"
#include "include/util.h"
#include "include/DistanceSense.h"
#include "include/LineSensor.h"

//Controls block sweep and robot movement from arriving at the cross to picking up the block.
BlockSweep::SweepState BlockSweep::BlockSwp(MotorControl &Mcon, DistanceSense &Dsense, WifiDebug &Debug){
    //If block sweep has been running for too long, time out and skip to block pickup
    if(timeout>BLOCK_SWEEP_TIMEOUT){//time out if there is a problem
        sweep_state=GRAB_BLOCK;
    }
    timeout+=10;
    unsigned long milli = millis(); //get the current time
    //read IR distance
    int IR_distance=Dsense.ReadIRDistance();

    //State based descision tree. This is is chronological order
    if (sweep_state == ROTATE_TO_OFFSET){//BlockSwp starts on the cross at the far side of the board
        timeout = 0;
        //Turns robot to be perpendicular to cross
        if (Mcon.TurnSetAngle(90, ANTI_CLOCKWISE) == COMPLETE){
            sweep_state = MOVE_OFFSET;
        }
    } else if (sweep_state == MOVE_OFFSET){
        //Moves robot back an offset to eliminate the error caused by block being too close for the IR sensor
        if (Mcon.MoveSetDistance(-8)== COMPLETE){
            sweep_state = ROTATE_TO_SWEEP_START;
        }
    } else if (sweep_state == ROTATE_TO_SWEEP_START){
        //Rotates robot to face sweep start location
        if (Mcon.TurnSetAngle(90, CLOCKWISE)== COMPLETE){
            sweep_state = START_SWEEP;
            starttime = milli; //set starting sweep time
            //reset other variables
            firstdetect=0;
            angleofblock=0;
            blockdetected=false;
            Debug.SendMessage("starting sweep");
            Mcon.ResetMovement();
        }
    } else if (sweep_state == START_SWEEP){
        //Turns robot 180 degrees whilst recording if block is sensed
        if (Mcon.TurnSetAngle(180, ANTI_CLOCKWISE)== COMPLETE){
            //did not find block, return to beginning
            sweep_state = ROTATE_TO_SWEEP_START;
            Mcon.TurnSetAngle(180, CLOCKWISE);
        }
        //measure ultrasound distance
        int ultrasoundDist=Dsense.ReadUltrasoundDistance();
        //if within ultrasound range, skip to move into magnet sensing range
        if(ultrasoundDist<ULTRASOUND_BLOCK_DETECTION_THRESHOLD && ultrasoundDist!=INVALID_READING){
            Debug.SendMessage("ultrasound takeover: "+String(ultrasoundDist));
            sweep_state = MOVE_INTO_MAGNET;
            Mcon.ResetMovement();
            Mcon.MoveSetDistance(-1);
        }
        if (!blockdetected){// if block has not yet been detected
        //will trigger if block is placed closer than any part of the ramp/ tunnel, if not a linear distance equation is need
            if (IR_distance < MIN_WALL_DISTANCE && IR_distance != INVALID_READING){
                //records time at which block is first detected
                blockdetected = true;
                firstdetect = milli;
                Debug.SendMessage("found block");
            }
        } else {
            //Triggers if block is no longer detected
            if ((IR_distance > MIN_WALL_DISTANCE || IR_distance==INVALID_READING) && firstdetect>0){
                blockdetected = false;
                //calculate what angle overshoot there is from facing the centre of the block
                overshoot = Mcon.TimeToAngleCon(int ((milli - firstdetect)/2));
                if(overshoot<1){//if overshoot is small then neglect
                    overshoot=0;
                }
                Debug.SendMessage("overshot: "+ String(overshoot) +" degrees");
                //calculate the angle between the block direction and the line
                if(angleofblock==0){
                    angleofblock = Mcon.TimeToAngleCon(milli-starttime)-overshoot;
                }
                //rotate back to the centre of the block
                sweep_state = ROTATE_TO_BLOCK;
                Mcon.ResetMovement();
                Mcon.SetMotors(0,0);
                
            } 
        }
    } else if (sweep_state == ROTATE_TO_BLOCK){
        //Turns robot to face midpoint of block 
        if (Mcon.TurnSetAngle(overshoot-Angle_Constant, CLOCKWISE)== COMPLETE){
            Debug.SendMessage("locked on");
            sweep_state = WAIT_FOR_IRSENSOR;
            Mcon.ResetMovement();
            Mcon.SetMotors(0,0);
            starttime = milli; //start timer
        }
    } else if (sweep_state == WAIT_FOR_IRSENSOR){//Stop for IR_WAIT_TIME for the robot to stabalise before measuring distance
        if (milli > starttime + IR_WAIT_TIME){
            //also read ultrasound distance
            float uDistance=Dsense.ReadUltrasoundDistance();
            if(IR_distance==INVALID_READING && uDistance==INVALID_READING){
                //if nothing is detected on either sensor, turn until something is found
                if(Mcon.TurnSetAngle(10,true)==COMPLETE){
                    Mcon.TurnSetAngle(10,false);
                }
            }else{
                if(IR_distance==INVALID_READING){ //if no valid IR reading, use ultrasound instead
                    blockdistance=uDistance;
                } else {
                    blockdistance = IR_distance;
                }
                //if block within the accurate range for IR, move straight towards block
                if(blockdistance<=ACCURATE_MEASURING_DISTANCE+5){
                    Debug.SendMessage("Accurate: "+String(blockdistance));
                    Mcon.ResetMovement();
                    Mcon.MoveSetDistance(blockdistance - ULTRASOUND_BLOCK_DETECTION_THRESHOLD);
                    sweep_state = MOVE_TOWARDS_MAGNET;
                    
                } else{
                    if(blockdistance > MIN_WALL_DISTANCE){ //if block cannot be found, sweep back to start position
                        sweep_state = ROTATE_TO_SWEEP_START;
                        Debug.SendMessage("lost block");
                        Mcon.ResetMovement();
                        Mcon.TurnSetAngle(angleofblock,true);
                    } else {//otherwise if block too far away, move to a distance where the IR reading is more accurate
                        sweep_state = MOVE_TO_ACCURATE_MEASURE_POINT;
                        Mcon.ResetMovement();
                        Mcon.MoveSetDistance(blockdistance - ACCURATE_MEASURING_DISTANCE);
                        Debug.SendMessage("Move closer: "+String(blockdistance));
                    }
                }
            }
        } else {
            Mcon.SetMotors(0,0);//stop motors while waiting
        }
    } else if (sweep_state == MOVE_TO_ACCURATE_MEASURE_POINT){
        //move closer to block to get a more accurate reading
        if (Mcon.MoveSetDistance(IR_distance - ACCURATE_MEASURING_DISTANCE)==COMPLETE){
            sweep_state = WAIT_FOR_IRSENSOR;
            starttime = milli; //start timer
            Mcon.ResetMovement();
            Mcon.SetMotors(0,0);
        }
        //just in case it reaches the block early, keep an eye on the ultrasound distance
        int ultrasoundDist=Dsense.ReadUltrasoundDistance();
        if(ultrasoundDist<ULTRASOUND_BLOCK_DETECTION_THRESHOLD && ultrasoundDist!=INVALID_READING){
            Debug.SendMessage("ultrasound takeover: "+String(ultrasoundDist));
            sweep_state = MOVE_INTO_MAGNET;
            Mcon.ResetMovement();
            Mcon.MoveSetDistance(3);
        }
    } else if(sweep_state == MOVE_TOWARDS_MAGNET){// this state is when the robot is moving close to the block
        if (Mcon.MoveSetDistance(IR_distance - ULTRASOUND_BLOCK_DETECTION_THRESHOLD)==COMPLETE){
            sweep_state = DETECT_MAGNET;
            starttime = milli;//start timer
            Mcon.ResetMovement();
            Mcon.SetMotors(0,0);
        }
        //just in case it reaches the block early, keep an eye on the ultrasound distance
        int ultrasoundDist=Dsense.ReadUltrasoundDistance();
        if(ultrasoundDist<DISTANCE_MEASURE_MAGNET+4 && ultrasoundDist!=INVALID_READING){
            Debug.SendMessage("ultrasound takeover: "+String(ultrasoundDist));
            sweep_state = MOVE_INTO_MAGNET;
            Mcon.ResetMovement();
            Mcon.MoveSetDistance(3);
        }
    }else if(sweep_state == MOVE_INTO_MAGNET){//move the last few cm ignoring sensors as they are not reliable at this range
        if(Mcon.MoveSetDistance(ULTRASOUND_BLOCK_DETECTION_THRESHOLD-DISTANCE_MEASURE_MAGNET)==COMPLETE){
            Debug.SendMessage("Final range: "+String(Dsense.ReadUltrasoundDistance(),2));
            Mcon.ResetMovement();
            Mcon.SetMotors(0,0);
            starttime=milli;//start timer
            sweep_state=DETECT_MAGNET;
        }
    }else if (sweep_state == DETECT_MAGNET){//wait while magnet LED is illuminated (this is done in main.ino)
        if(milli-starttime>5000){
            Mcon.ResetMovement();
            Mcon.SetMotors(0,0);
            sweep_state =GRAB_BLOCK;// block grabbing is handled within main.ino, so this is the end point of BlockSweep
            //calculate angle to turn back to line
            crossangle = int((180 * CROSS_OFFSET * asin(sin(((90-angleofblock)*PI)/180)/blockdistance))/ PI);
        }
    }
    return sweep_state;
}

//Controls robot movement from picking up the block to returning to the line
BlockSweep::SweepState BlockSweep::ReturnToCross(MotorControl &Mcon, DistanceSense &Dsense, LineSensor &Lsense, WifiDebug &Debug){
    if (sweep_state == GRAB_BLOCK){
        sweep_state = REVERSE_TO_LINE;
        Mcon.ResetMovement();
    } else if (sweep_state == REVERSE_TO_LINE){
        if(Mcon.MoveSetDistance(-200)==COMPLETE){//reverse 2 meters
        //if this completes then the robot is lost as it should reach the line within 2 meters
            sweep_state = LINE_NOT_FOUND;//main.ino will handle this state
            Mcon.ResetMovement();
        }
        if(Lsense.juntionDetect()){//junction sensor used for line detection
            Mcon.SetMotors(0,0);
            sweep_state = ROTATE_FORWARD;
            Mcon.ResetMovement();
        }
    } else if (sweep_state == ROTATE_FORWARD){
        //Rotates robot back to face along the line
        if(!Mcon.TurnSetAngle(angleofblock, CLOCKWISE)){
            sweep_state = SWEEP_COMPLETE;
            Mcon.ResetMovement();
        }
    }
    return sweep_state;
}
