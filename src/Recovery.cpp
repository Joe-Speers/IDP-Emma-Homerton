#include "include/MotorControl.h"
#include "include/WifiDebug.h"
#include "include/Recovery.h"
#include "include/util.h"
#include "include/LineSensor.h"
#include "include/DistanceSense.h"


Recovery::RecoveryState Recovery::blockSite(MotorControl Mcon, WifiDebug Debug,LineSensor Lsense, DistanceSense Dsense){
    milli = millis();
    junct_cur = Lsense.juntionDetect();
    distance = Dsense.ReadIRDistance();
    if (LastState==RECOVERY_SETUP){
        start_timer = milli;
        closest_distance = 1000;
        Mcon.ResetMovement();
        Debug.SendMessage("Finding closest wall");
    }
    if (LastState==WALL_FIND){
        if (Mcon.TurnSetAngle(360, CLOCKWISE)){
            if (distance < closest_distance){
                wall_detect_time = milli;
                closest_distance = distance;
            }
        }
        else{
            wall_angle = Mcon.TimeToAngleCon(wall_detect_time);
            LastState=ROTATE_TO_WALL;
            Mcon.ResetMovement();
            Debug.SendMessage("Rotating to face wall");
        }
    }
    if (LastState==ROTATE_TO_WALL){
        if (!Mcon.TurnSetAngle(wall_angle, CLOCKWISE)){
            if (closest_distance <= 25){
                LastState = DISTANCE_TOO_SMALL;
            }
            else{
                LastState = FIND_LINE_BACKWARDS;
                
            }
            Mcon.ResetMovement();
            //LastState = FIND_LINE_BACKWARDS; i presume this is not meant to be here
            Debug.SendMessage("Moving to find line");
        }
    }
    if (LastState==FIND_LINE_BACKWARDS){
        if (distance < 29){
            if (junct_prev == false && junct_cur == true){
                Mcon.SetMotors(0,0);
                LastState = LINE_FOUND;
                Mcon.ResetMovement();
                Debug.SendMessage("Line has been found");
            }
            else if(!Mcon.MoveSetDistance(closest_distance - 120)){
                LastState = LINE_NOT_FOUND;
                Mcon.ResetMovement();
                Debug.SendMessage("Line not found rotating 90 degrees");
            }
        }
        if (closest_distance >= 29 && closest_distance <= 100){
            if (junct_prev == false && junct_cur == true){
                Mcon.SetMotors(0,0);
                LastState = LINE_FOUND;
                Mcon.ResetMovement();
                Debug.SendMessage("Line has been found");
            }
            if(!Mcon.MoveSetDistance(closest_distance-110)){
                LastState = FIND_LINE_FORWARDS;
                Mcon.ResetMovement();
            }
        }
        if (closest_distance > 100){
            LastState = FIND_LINE_FORWARDS;
            Mcon.ResetMovement();
        }
    }

    if (LastState==FIND_LINE_FORWARDS){
        if (closest_distance >= 29 && closest_distance <= 100){
            if (junct_prev == false && junct_cur == true){
                Mcon.SetMotors(0,0);
                LastState = LINE_FOUND;
                Mcon.ResetMovement();
                Debug.SendMessage("Line has been found");
            }
            if(!Mcon.MoveSetDistance(85)){
                LastState = LINE_NOT_FOUND;
                Mcon.ResetMovement();
            }
        }
        if (closest_distance > 100){
            if (junct_prev == false && junct_cur == true){
                Mcon.SetMotors(0,0);
                LastState = LINE_FOUND;
                Debug.SendMessage("Line has been found");
                Mcon.ResetMovement();
            }
            if (!Mcon.MoveSetDistance(closest_distance-25)){
                LastState = LINE_NOT_FOUND;
                Mcon.ResetMovement();
            }
        }
    }

    if (LastState==LINE_NOT_FOUND){
        if(!Mcon.TurnSetAngle(90, CLOCKWISE)){
            LastState = RECOVERY_SETUP;
            Mcon.ResetMovement();
        }
    }

    if (LastState==LINE_FOUND){

    }
    if (LastState==DISTANCE_TOO_SMALL){
        if (!Mcon.MoveSetDistance(-5)){
            LastState = RECOVERY_SETUP;
            Mcon.ResetMovement();
        }
    }
    return LastState;
    
    junct_prev = Lsense.juntionDetect();

}

Recovery::RecoveryState Recovery::withblock(MotorControl Mcon, WifiDebug Debug,LineSensor Lsense, Location location, Purpose purpose, int usdistance){

    return LastState;

}

Recovery::RecoveryState Recovery::ramp(MotorControl Mcon, WifiDebug Debug,LineSensor Lsense, Location location, Purpose purpose, int usdistance){

    return LastState;

}

Recovery::RecoveryState Recovery::tunnel(MotorControl Mcon, WifiDebug Debug,LineSensor Lsense, Location location, Purpose purpose, int usdistance){

    return LastState;

}