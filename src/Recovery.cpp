#include "include/MotorControl.h"
#include "include/WifiDebug.h"
#include "include/Recovery.h"
#include "include/util.h"
#include "include/LineSensor.h"


Recovery::RecoveryState Recovery::blockSite(MotorControl Mcon, WifiDebug Debug,LineSensor Lsense, Location location, Purpose purpose, int distance){
    milli = millis();
    junct_cur = Lsense.juntionDetect();
    if (LastState==RECOVERY_SETUP){
        start_timer = milli;
        closest_distance = 1000;
        Debug.SendMessage("Finding closest wall");
    }
    if (LastState==LOCATION_FIND){
        if (Mcon.TurnSetAngle(360, CLOCKWISE)){
            if (distance < closest_distance){
                wall_detect_time = milli;
          }
        }
        else{
            wall_angle = Mcon.TimeToAngleCon(wall_detect_time);
            LastState=ROTATE_TO_WALL;
            Debug.SendMessage("Rotating to face wall");
        }
    }
    if (LastState==ROTATE_TO_WALL){
        if (!Mcon.TurnSetAngle(wall_angle, CLOCKWISE)){
            if (closest_distance <= 23){
                LastState = DISTANCE_TOO_SMALL;
            }
            else{
                LastState = FIND_LINE;
            }
            LastState = FIND_LINE;
            Debug.SendMessage("Moving to find line");
        }
    }
    if (LastState==FIND_LINE){
        if (distance < 30){
            if (junct_prev == false && junct_cur == true){
                Mcon.SetMotors(0,0);
                LastState = LINE_FOUND;
                Debug.SendMessage("Line has been found");
            }
            else if(!Mcon.MoveSetDistance(120-start_distance)){
                Debug.SendMessage("Line not found rotating 90 degrees");
            }
        }
        if (distance > 120){
            if (Lsense.isLineDetected()){
                Mcon.SetMotors(0,0);
                LastState = LINE_FOUND;
            }
            else if(!Mcon.MoveSetDistance(start_distance-30)){
                Debug.SendMessage("Line not found rotating 90 degrees");
            }
        }
    }
    if (LastState==LINE_FOUND){

    }
    if (LastState==DISTANCE_TOO_SMALL){
        if (!Mcon.MoveSetDistance(-5)){
            LastState = RECOVERY_SETUP;
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