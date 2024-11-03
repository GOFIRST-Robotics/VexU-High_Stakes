#pragma once
#include "lemlib/api.h"


#define CHASIS_LENGTH
#define TRACK_WIDTH
#define GEAR_RATIO
#define HORIZONTAL_DRIFT 2
#define DRIVETRAIN_RPM 480 //This is for drive train itself, not the individual motors
#define WHEEL_DIAMETER lemlib::Omniwheel::NEW_325 //This may be wrong as I don't know if it is half or not, or if it is old or new
#define FRONT_LEFT_PORT 1 //Ports for the motors in the drivetrain
#define FRONT_RIGHT_PORT 2
#define MID_LEFT_PORT 3
#define MID_RIGHT_PORT 4
#define BACK_LEFT_PORT 5
#define BACK_RIGHT_PORT 6

//-1 for a reversed motor 1 for a forward motor
#define FRONT_LEFT_DIRECTION 1 //Ports for the motors in the drivetrain
#define FRONT_RIGHT_DIRECTION 1 
#define MID_LEFT_DIRECTION 1
#define MID_RIGHT_DIRECTION 1
#define BACK_LEFT_DIRECTION 1
#define BACK_RIGHT_DIRECTION 1
