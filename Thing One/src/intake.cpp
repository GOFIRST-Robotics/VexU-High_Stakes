#include "intake.h"

pros::Motor* intake::intakeMotor = new pros::Motor(CLIMB_MOTOR_PORT, CLIMB_MOTOR_GEAR_RATIO, pros::v5::MotorEncoderUnits::degrees);
controller*  intake::robotController = controller::getInstance();

void intake::init() {
    
}

void intake::periodic() {
    //Basically just look for a button press to run the motor, or not, I'm not your boss
    if(robotController->get_digital(INTAKE_BUTTON)) {
        intake::intakeMotor->move(INTAKE_MOTOR_SPEED);

    } else {
        intake::intakeMotor->brake();
    }

}