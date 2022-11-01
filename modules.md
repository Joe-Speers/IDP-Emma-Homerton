# Modules and Descision system
## Modules
- Main
    - Creates objects
    - Setup script
    - Descision system (joe to set up, then everyone)
- Wifi Debug
- MotorControl (jerry + joe)
  - Servos
  - Motors
- LineSense (joe + jerry+ kyle)
    - Junction detection (kyle)
    - line following
- BlockCapture (orla + jerry + kyle)
    - Sensing block
    - senses block for magnet
    - displays LED
    - picking up block
- DistanceSense (orla)
    - Ultrasound
    - Infrared
    -distance calculation
- Recovery (kyle)
- AdditonalSensors (kyle)
  - Tunnel sensor
  - Ramp sensor
- BlockSweep
-TunnelSensor
TiltSensor

# Descision System
The descision system executes every loop, with an interval of `tick_time`

State enumerables:
 - Location
 - Purpose: What the robot is trying to achieve
 - Task: The specific current action.

 Timing variables:
 - s (int): time in seconds since robot started
 - m (int): millisecond counter from 0-999 then loops every second back to zero. In increments of `tick_time` currently set to 10ms.


# Recovery Module
  ## Inputs
  - location
  - purpose
  - task
  - time in seconds (potentially millliseconds also)
  - linesensors
  - linefollow sensors
  - junction sensor
  - distance sensors
  - ultrasound
  - IR

  ## Outputs
  - MotorControl

  ## Location Dependent Submodules

  Block collection area:
  - rotate on spot (may need to reverse first)
  - locate gaps in central obstacle
  - calculate allignment
  - rotate until perpendicular to line
  - forward movement line is detected
  - allign robot with line via rotation
  - set task according to purpose?
  - possibly returning robot to previous tasks starting location?
      
    concerns:
    - accidental collisions with block

  Starting side:
  - rotate on spot (may need to reverse first)
  - locate gaps in central obstacle
  - calculate allignment
  - rotate until perpendicular with line and facing line
  - forward movement perpendicular to line until line is detected
  - allign robot with line via rotation
  - set task according to purpose? 
  - possibly returning robot to previous tasks starting location?

    concerns:
    - Line detecting box instead of main line

  Tunnel:
  -rotate on spot (may need to reverse first)
  -locate gaps in central obstacle
  -calculate allignment
  -rotate to be paralel with tunnel
  -set forward movement

    concerns:
    -robot still stuck to wall

  Ramp:
  designed only considering robot veering close to wall
  - reverse until linesense detects the line 
    
  concerns:
  - falling off ramp
  - a loop of forwards and backwards movement

# block sweep module
 - Starts with robot on cross
 - Rotates 180 degrees anticlockwise
 - while rotating logs angle block is detected and angle block is no longer detected
 - rotates back to midpoint of these angles
 - measures distance
 - moves forward to that distance minus some small constant
 - iniates grab sequence
 - reverses distance from before
 - rotates back to line face forwards
 - set tasks to line follow

