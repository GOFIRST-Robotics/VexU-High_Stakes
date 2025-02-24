#include "drive.h"
#include "main.h"


void robotDrive::showEncoderDetails() {

    // this runs in its own thread
    
    while (true) { // infinite loop
        // print measurements from the rotation sensor
        pros::lcd::print(1, "Vertical Sensor: %i", vPod.get_position());
        pros::lcd::print(2, "Horizontal Sensor: %i", hPod.get_position());
        pros::lcd::print(3, "X: %f", chassis.getPose().x);
        pros::lcd::print(4, "Y: %f", chassis.getPose().y);
        pros::lcd::print(5, "Theta: %f", chassis.getPose().theta);
        pros::delay(20); // delay to save resources. DO NOT REMOVE
    }

}


void robotDrive::teleopRoutine() {
    this->chassis.arcade(pros::E_CONTROLLER_ANALOG_LEFT_Y, pros::E_CONTROLLER_ANALOG_LEFT_X, false, 0.75);
}

void robotDrive::init(){
    pros::Task screenupdate([this] {this->showEncoderDetails();});
}