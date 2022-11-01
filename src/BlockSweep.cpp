#include "include/BlockSweep.h"
#include "include/Motorcontrol.h"
#include "include/util.h"
#include "include/DistanceSense.h"
#include "include/LineSensor.h"



// free RAM check for debugging. SRAM for ATmega328p = 2048Kb.
int availableMemory() {
    // Use 1024 with ATmega168
    int size = 6144;
    byte *buf;
    while ((buf = (byte *) malloc(--size)) == NULL);
        free(buf);
    return size;
}
BlockSweep::SweepState BlockSweep::BlockSwp(MotorControl &Mcon, DistanceSense &Dsense, WifiDebug &Debug){
    //Serial.println(availableMemory());
    unsigned long milli = millis();
    distance=Dsense.ReadIRDistance();
    if (laststate == ROTATE_TO_OFFSET){
        //Turns robot to be perpendicular to cross
        if (Mcon.TurnSetAngle(90, ANTI_CLOCKWISE) == COMPLETE){
            laststate = MOVE_OFFSET;
        }
    } else if (laststate == MOVE_OFFSET){
        //Moves robot back an offset to eliminate the error caused by block being too close for the IR sensor
        if (Mcon.MoveSetDistance(-CROSS_OFFSET)== COMPLETE){
            laststate = ROTATE_TO_SWEEP_START;
        }
    } else if (laststate == ROTATE_TO_SWEEP_START){
        //Rotates robot to face sweep start location
        if (Mcon.TurnSetAngle(90, CLOCKWISE)== COMPLETE){
            laststate = START_SWEEP;
            starttime = milli;
            firstdetect=0;
            blockdetected=false;
            Debug.SendMessage("swp");
            Mcon.ResetMovement();
        }
    } else if (laststate == START_SWEEP){
        //Turns robot 180 degrees whilst recording if block is sensed
        
        if (Mcon.TurnSetAngle(180, ANTI_CLOCKWISE)== COMPLETE){
            //did not find block, return to beginning
            laststate = ROTATE_TO_SWEEP_START;
            Mcon.TurnSetAngle(180, CLOCKWISE);
        }
        if (!blockdetected){
        //will work if block is placed closer than any part of the ramp/ tunnel, if not a linear distance equation is need
            if (distance < MIN_WALL_DISTANCE && distance != INVALID_READING){
                //records time at which block is first detected
                blockdetected = true;
                firstdetect = milli;
                Debug.SendMessage("found");
            }
        } else {
            if ((distance > MIN_WALL_DISTANCE || distance==INVALID_READING) && firstdetect>0){
                //records time at which block is no longer detected
                blockdetected = false;
                overshoot = Mcon.TimeToAngleCon(int ((milli - firstdetect)/2));
                if(overshoot<0){//probably a false reading! Redo this reading
                    Debug.SendMessage("anomoly");
                    blockdetected = false;
                    return;
                }
                Debug.SendMessage("overshot: "+ String(overshoot) +" degrees");
                angleofblock = Mcon.TimeToAngleCon(milli-starttime)-overshoot;
                laststate = ROTATE_TO_BLOCK;
                Mcon.SetMotors(0,0);
                Mcon.ResetMovement();
            } 
        }
    } else if (laststate == ROTATE_TO_BLOCK){
        //Turns robot to face midpoint of block and records distance to block
        if (Mcon.TurnSetAngle(overshoot-Angle_Constant, CLOCKWISE)== COMPLETE){
            Debug.SendMessage("locked on");
            laststate = WAIT_FOR_IRSENSOR;
            starttime = milli;
        }
    } else if (laststate == WAIT_FOR_IRSENSOR){
        Mcon.SetMotors(0,0);
        if(distance==INVALID_READING){ //go back to sweeping
            Mcon.ResetMovement();
            laststate = ROTATE_TO_SWEEP_START;
            Mcon.TurnSetAngle(10, CLOCKWISE);
        }
        if (milli > starttime + IR_WAIT_TIME && distance!=INVALID_READING){
            blockdistance = distance;
            
            
            if(blockdistance<=ACCURATE_MEASURING_DISTANCE+5){
                Debug.SendMessage("Accurate: "+String(blockdistance));
                Mcon.MoveSetDistance(blockdistance - ULTRASOUND_BLOCK_DETECTION_THRESHOLD+5);
                laststate = MOVE_TO_WITHIN_ULTRASOUND_RANGE;
                Mcon.ResetMovement();
            } else{
                laststate = MOVE_TO_ACCURATE_MEASURE_POINT;
                Mcon.MoveSetDistance(blockdistance - ACCURATE_MEASURING_DISTANCE);
                Debug.SendMessage("Move closer: "+String(blockdistance));
                Mcon.ResetMovement();
            }
            
        }
    } else if (laststate == MOVE_TO_ACCURATE_MEASURE_POINT){
        if(distance > MIN_WALL_DISTANCE){ //if lost block
            Mcon.ResetMovement();
            laststate = ROTATE_TO_SWEEP_START;
            Mcon.TurnSetAngle(5, CLOCKWISE);//only rotate 5 degrees
        }
        if (Mcon.MoveSetDistance(blockdistance - ACCURATE_MEASURING_DISTANCE)==COMPLETE){
            laststate = WAIT_FOR_IRSENSOR;
            starttime = milli;
            Mcon.SetMotors(0,0);
        }
        //just in case it reaches the block early, keep an eye on the ultrasound distance
        int ultrasoundDist=Dsense.ReadUltrasoundDistance();
        if(ultrasoundDist<ULTRASOUND_BLOCK_DETECTION_THRESHOLD && ultrasoundDist!=INVALID_READING){
            Debug.SendMessage("ultrasound takeover: "+String(ultrasoundDist));
            laststate = PAUSE_BEFORE_FINAL_APPROACH;
            starttime=milli;
            Mcon.ResetMovement();
        }
        
    }else if (laststate == MOVE_TO_WITHIN_ULTRASOUND_RANGE){
        if(distance>ACCURATE_MEASURING_DISTANCE){
            //error lost block
        }
        //Moves robot forwards to within ultrasound range
        if (Mcon.MoveSetDistance(blockdistance - ULTRASOUND_BLOCK_DETECTION_THRESHOLD+5)==COMPLETE){          
            laststate = PAUSE_BEFORE_FINAL_APPROACH;
            starttime=milli;
        }
        int ultrasoundDist=Dsense.ReadUltrasoundDistance();
        if(ultrasoundDist<ULTRASOUND_BLOCK_DETECTION_THRESHOLD && ultrasoundDist!=INVALID_READING){
            Debug.SendMessage("ultrasound takeover: "+String(ultrasoundDist));
            laststate = PAUSE_BEFORE_FINAL_APPROACH;
            starttime=milli;
            Mcon.ResetMovement();
        }
    } else if (laststate == PAUSE_BEFORE_FINAL_APPROACH){
        Mcon.SetMotors(0,0);
        if(milli-starttime>500){
            laststate=MOVE_INTO_MAGNET;
            int ultrasoundDist=Dsense.ReadUltrasoundDistance();
            Debug.SendMessage("Stable reading: "+String(ultrasoundDist));
            Mcon.ResetMovement();
            return;
        }
    } else if(laststate == MOVE_INTO_MAGNET){
        if(Mcon.MoveSetDistance(Dsense.ReadUltrasoundDistance()-DISTANCE_MEASURE_MAGNET)==COMPLETE){//wait for previous task to complete
            Debug.SendMessage("Final range: "+String(Dsense.ReadUltrasoundDistance(),2));
            Mcon.SetMotors(0,0);
            starttime=milli;
            laststate=DETECT_MAGNET;
            
        }
    }else if (laststate == DETECT_MAGNET){
        if(milli-starttime>5000){
            Mcon.SetMotors(0,0);
            laststate =GRAB_BLOCK; //done!
            //find out angle to turn back
            crossangle = int((180 * CROSS_OFFSET * asin(sin(((90-angleofblock)*3.141592)/180)/blockdistance))/ 3.141592);
            crossdistance = int((sin(((90-angleofblock)*3.141592)/180)*CROSS_OFFSET)/sin(((crossangle)*3.141592)/180)); 
        }
    }
    return laststate;
}

BlockSweep::SweepState BlockSweep::ReturnToCross(MotorControl &Mcon, DistanceSense &Dsense, LineSensor &Lsense, WifiDebug &Debug){

    unsigned long milli = millis();

    if (laststate == GRAB_BLOCK){
        laststate = ROTATE_TO_CROSS;
        Mcon.ResetMovement();
    }
    if (laststate == ROTATE_TO_CROSS){
        //Turns robot around
        if (angleofblock > 90){
            if (Mcon.TurnSetAngle(180 - crossangle, ANTI_CLOCKWISE)== COMPLETE){
                Debug.SendMessage("returning to cross");
                laststate = MOVE_TO_CROSS;
            }
        }
        else {
            if (Mcon.TurnSetAngle(180 - crossangle, CLOCKWISE)== COMPLETE){
                laststate = MOVE_TO_CROSS;
                Debug.SendMessage("returning to cross");
            }
        }
    }
    if (laststate == MOVE_TO_CROSS){
        //Moves robot back to cross
        Mcon.SetMotors(255,255);
        if (Lsense.juntionDetect()){
            laststate = ROTATE_FORWARD;
            Debug.SendMessage("rotating forward");
            Mcon.ResetMovement();
        }
    }
    if (laststate == ROTATE_FORWARD){
        //Rotates robot to face allong the line
        if (angleofblock > 90){
            if (Mcon.TurnSetAngle(180 - crossangle - angleofblock, ANTI_CLOCKWISE)== COMPLETE ||  Lsense.isLineDetected()){
                laststate = SWEEP_COMPLETE;
                Debug.SendMessage("complete");
                Mcon.ResetMovement();
            }
        }
        else {
            if (Mcon.TurnSetAngle(180 + crossangle - angleofblock, ANTI_CLOCKWISE)== COMPLETE||  Lsense.isLineDetected()){
                laststate = SWEEP_COMPLETE;
                Debug.SendMessage("complete");
                Mcon.ResetMovement();
            }
        }
    }
    return laststate;
}
