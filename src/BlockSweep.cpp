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
        if (Mcon.MoveSetDistance(-1)== COMPLETE){
            laststate = ROTATE_TO_SWEEP_START;
        }
    } else if (laststate == ROTATE_TO_SWEEP_START){
        //Rotates robot to face sweep start location
        if (Mcon.TurnSetAngle(90, CLOCKWISE)== COMPLETE){
            laststate = START_SWEEP;
            starttime = milli;
            firstdetect=0;
            angleofblock=0;
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
        int ultrasoundDist=Dsense.ReadUltrasoundDistance();
        if(ultrasoundDist<DISTANCE_MEASURE_MAGNET+4 && ultrasoundDist!=INVALID_READING){
            Debug.SendMessage("ultrasound takeover: "+String(ultrasoundDist));
            laststate = MOVE_INTO_MAGNET;
            starttime=milli;
            Mcon.ResetMovement();
            Mcon.MoveSetDistance(-1);
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
                    //Debug.SendMessage("anomoly");
                    //blockdetected = false;
                    //return;
                    overshoot=0;
                }
                Debug.SendMessage("overshot: "+ String(overshoot) +" degrees");
                if(angleofblock==0){
                    angleofblock = Mcon.TimeToAngleCon(milli-starttime)-overshoot;
                }
                laststate = ROTATE_TO_BLOCK;
                Mcon.ResetMovement();
                Mcon.SetMotors(0,0);
                
            } 
        }
    } else if (laststate == ROTATE_TO_BLOCK){
        //Turns robot to face midpoint of block and records distance to block
        if (Mcon.TurnSetAngle(overshoot, CLOCKWISE)== COMPLETE){
            Debug.SendMessage("locked on");
            laststate = WAIT_FOR_IRSENSOR;
            Mcon.ResetMovement();
            Mcon.SetMotors(0,0);
            starttime = milli;
        }
    } else if (laststate == WAIT_FOR_IRSENSOR){
        if (milli > starttime + IR_WAIT_TIME){
            float uDistance=Dsense.ReadUltrasoundDistance();
            if(distance==INVALID_READING && uDistance==INVALID_READING){
                if(Mcon.TurnSetAngle(10,true)==COMPLETE){
                    Mcon.TurnSetAngle(10,false);
                }
            }else{
                if(distance==INVALID_READING){
                    blockdistance=uDistance;
                } else {
                    blockdistance = distance;
                }
                if(blockdistance<=ACCURATE_MEASURING_DISTANCE+5){
                    Debug.SendMessage("Accurate: "+String(blockdistance));
                    Mcon.ResetMovement();
                    Mcon.MoveSetDistance(blockdistance - DISTANCE_MEASURE_MAGNET +3);
                    laststate = MOVE_TOWARDS_MAGNET;
                    
                } else{
                    laststate = MOVE_TO_ACCURATE_MEASURE_POINT;
                    Mcon.ResetMovement();
                    Mcon.MoveSetDistance(blockdistance - ACCURATE_MEASURING_DISTANCE);
                    Debug.SendMessage("Move closer: "+String(blockdistance));
                    
                }
            }
            
        } else {
            Mcon.SetMotors(0,0);
        }
    } else if (laststate == MOVE_TO_ACCURATE_MEASURE_POINT){
        //if(distance > MIN_WALL_DISTANCE && false){ //if lost block
        //    Mcon.ResetMovement();
        //   laststate = ROTATE_TO_SWEEP_START;
        //    Mcon.TurnSetAngle(5, CLOCKWISE);//only rotate 5 degrees
        //}
        if (Mcon.MoveSetDistance(blockdistance - ACCURATE_MEASURING_DISTANCE)==COMPLETE){
            laststate = WAIT_FOR_IRSENSOR;
            starttime = milli;
            Mcon.ResetMovement();
            Mcon.SetMotors(0,0);
        }
        //just in case it reaches the block early, keep an eye on the ultrasound distance
        int ultrasoundDist=Dsense.ReadUltrasoundDistance();
        if(ultrasoundDist<DISTANCE_MEASURE_MAGNET+4 && ultrasoundDist!=INVALID_READING){
            Debug.SendMessage("ultrasound takeover: "+String(ultrasoundDist));
            laststate = MOVE_INTO_MAGNET;
            starttime=milli;
            Mcon.ResetMovement();
            Mcon.MoveSetDistance(3);
        }
        
    } else if(laststate == MOVE_TOWARDS_MAGNET){
        //if(distance > MIN_WALL_DISTANCE && false){ //if lost block
        //    Mcon.ResetMovement();
        //    laststate = ROTATE_TO_SWEEP_START;
        //    Mcon.TurnSetAngle(5, CLOCKWISE);//only rotate 5 degrees
        //}
        if (Mcon.MoveSetDistance(blockdistance - DISTANCE_MEASURE_MAGNET +3)==COMPLETE){
            Debug.SendMessage("used IR");
            laststate = DETECT_MAGNET;
            starttime = milli;
            Mcon.ResetMovement();
            Mcon.SetMotors(0,0);
        }
        //just in case it reaches the block early, keep an eye on the ultrasound distance
        int ultrasoundDist=Dsense.ReadUltrasoundDistance();
        if(ultrasoundDist<DISTANCE_MEASURE_MAGNET+4 && ultrasoundDist!=INVALID_READING){
            Debug.SendMessage("ultrasound takeover: "+String(ultrasoundDist));
            laststate = MOVE_INTO_MAGNET;
            starttime=milli;
            Mcon.ResetMovement();
            Mcon.MoveSetDistance(3);
        }
    }else if(laststate == MOVE_INTO_MAGNET){
        if(Mcon.MoveSetDistance(3)==COMPLETE){//wait for previous task to complete
            Debug.SendMessage("Final range: "+String(Dsense.ReadUltrasoundDistance(),2));
            Mcon.ResetMovement();
            Mcon.SetMotors(0,0);
            starttime=milli;
            laststate=DETECT_MAGNET;
            
        }
    }else if (laststate == DETECT_MAGNET){
        if(milli-starttime>5000){
            Mcon.ResetMovement();
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

    if (laststate == GRAB_BLOCK){
        laststate = REVERSE_TO_LINE;
        Mcon.ResetMovement();
    }
    if (laststate == REVERSE_TO_LINE){
        //Turns robot around
        if(!Mcon.MoveSetDistance(-200)){
            laststate = LINE_NOT_FOUND;
            Mcon.ResetMovement();
        }
        if(Lsense.juntionDetect()){
            Mcon.SetMotors(0,0);
            laststate = ROTATE_FORWARD;
            Mcon.ResetMovement();
        }
    }
    
    if (laststate == ROTATE_FORWARD){
        //Rotates robot to face allong the line
        if(!Mcon.TurnSetAngle(90, CLOCKWISE)){
            laststate = SWEEP_COMPLETE;
            Mcon.ResetMovement();
        }
    }

    if (laststate == LINE_NOT_FOUND){
        
    }
    return laststate;
}
