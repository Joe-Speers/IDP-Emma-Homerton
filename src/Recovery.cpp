#include "include/MotorControl.h"
#include "include/WifiDebug.h"
#include "include/Recovery.h"
#include "include/util.h"
#include "include/LineSensor.h"


Recovery::RecoveryState Recovery::blocksite(MotorControl Mcon, WifiDebug Debug,LineSensor Lsense, Location location, Purpose purpose, int usdistance){
    milli = millis();
    if (LastState==RECOVERY_SETUP){
        start_timer = milli;
        closest_distance = 1000;
        Debug.SendMessage("Finding closest wall");
    }
    if (LastState==LOCATION_FIND){
        if (Mcon.TurnSetAngle(360, CLOCKWISE)){
            if (usdistance < closest_distance){
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
            LastState = FIND_LINE;
            Debug.SendMessage("Moving to find line");
        }
    }
    if (LastState==FIND_LINE){
        if (usdistance < 30){
            if (Lsense.isLineDetected()){
                Mcon.SetMotors(0,0);
                LastState = LINE_FOUND;
            }
            else if(!Mcon.MoveSetDistance(120-start_distance)){
                Debug.SendMessage("Line not found rotating 90 degrees");
            }
        }
        if (usdistance > 120){
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
    return LastState;

}

Recovery::RecoveryState Recovery::start(MotorControl Mcon, WifiDebug Debug,LineSensor Lsense, Location location, Purpose purpose, int usdistance){

    return LastState;

}

Recovery::RecoveryState Recovery::ramp(MotorControl Mcon, WifiDebug Debug,LineSensor Lsense, Location location, Purpose purpose, int usdistance){

    return LastState;

}

Recovery::RecoveryState Recovery::tunnel(MotorControl Mcon, WifiDebug Debug,LineSensor Lsense, Location location, Purpose purpose, int usdistance){

    return LastState;

}