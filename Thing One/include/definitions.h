#pragma once
#include "lemlib/api.hpp"
#include "api.h"



#define CHASIS_LENGTH 1
#define TRACK_WIDTH 1
#define GEAR_RATIO 1
#define HORIZONTAL_DRIFT 2
#define DRIVETRAIN_RPM 480 //This is for drive train itself, not the individual motors
#define WHEEL_DIAMETER lemlib::Omniwheel::NEW_325 //This may be wrong as I don't know if it is half or not, or if it is old or new
#define FRONT_LEFT_PORT 1 //Ports for the motors in the drivetrain
#define FRONT_RIGHT_PORT 2
#define MID_LEFT_PORT 3
#define MID_RIGHT_PORT 4
#define BACK_LEFT_PORT 5
#define BACK_RIGHT_PORT 6
#define IMU_PORT_ONE 10 // this is a temperary number and should be condimed


//negative number for a reversed motor positive number for a forward motor
#define FRONT_LEFT_DIRECTION 1 //Ports for the motors in the drivetrain
#define FRONT_RIGHT_DIRECTION 1 //placeholder values
#define MID_LEFT_DIRECTION 1
#define MID_RIGHT_DIRECTION 1
#define BACK_LEFT_DIRECTION 1
#define BACK_RIGHT_DIRECTION 1


//Climber defs
#define CLIMB_MOTOR_PORT 14
#define CLIMB_MOTOR_GEAR_RATIO pros::v5::MotorGearset::green
#define CLIMB_MOTOR_SPEED 1

//Intake definitions
#define INTAKE_MOTOR_PORT 15
#define INTAKE_MOTOR_GEAR_RATiO pros::v5::MotorGearset::green
#define INTAKE_MOTOR_SPEED 0.4






//Control definitions
#define INTAKE_BUTTON pros::E_CONTROLLER_DIGITAL_L1 
#define CLIMB_EXTEND_BUTTON pros::E_CONTROLLER_DIGITAL_R1
#define CLIMB_RETRACT_BUTTON pros::E_CONTROLLER_DIGITAL_R2