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
- AdditonalSensors??
  - Tunnel sensor
  - Ramp sensor

# Descision System
The descision system executes every loop, with an interval of `tick_time`

State enumerables:
 - Location
 - Purpose: What the robot is trying to achieve
 - Task: The specific current action.

 Timing variables:
 - s (int): time in seconds since robot started
 - m (int): millisecond counter from 0-999 then loops every second back to zero. In increments of `tick_time` currently set to 10ms.

