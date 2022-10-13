/* 
Stores global constants and enums
*/

// Pin assignment

// constants


// =enums for State system
enum Location{
    START_SQUARE            = 0,
    DROPOFF_SIDE            = 1,
    RAMP                    = 2,
    COLLECTION_SIDE         = 3,
    CROSS                   = 4,
    BLOCK_COLLECTION_AREA   = 5,
    TUNNEL                  = 6,
    RED_SQUARE              = 7,
    GREEN_SQARE             = 8
};

enum Purpose{
    EXIT_START_BOX = 0,
    TRAVEL_TO_FAR_SIDE = 1, //add more as needed

};

enum Task{
    MOVE_FORWARD = 0,
    REVERSE = 1,
    TURN_LEFT = 2,
    TURN_RIGHT = 3,
    FOLLOW_LINE = 4,
    SLOW_SWEEP =5, //add more as needed

};

