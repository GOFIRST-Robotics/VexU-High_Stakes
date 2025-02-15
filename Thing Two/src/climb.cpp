#include "climb.h"

pros::Motor* climb::climbMotor = new pros::Motor(CLIMB_MOTOR_PORT, CLIMB_MOTOR_GEAR_RATIO, pros::v5::MotorEncoderUnits::degrees);
controller* climb::robotController = controller::getInstance();

void climb::init() {
    
}

void climb::periodic() {
    if(robotController->get_digital(CLIMB_EXTEND_BUTTON)) {
        climbMotor->move(CLIMB_MOTOR_SPEED);
    }
    if(robotController->get_digital(CLIMB_RETRACT_BUTTON)) {
        climbMotor->move(CLIMB_MOTOR_SPEED);
    }
}