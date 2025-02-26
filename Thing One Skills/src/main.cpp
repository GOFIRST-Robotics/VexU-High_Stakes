
//This is code for the 15" Robot, for the 24" go to Thing One

#include "main.h"
#include "lemlib/api.hpp"
//#include "../include/definitons.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);


// left motor group
pros::MotorGroup left_motor_group({-16, -14, 15}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({18, -19, 20}, pros::MotorGears::blue);

pros::MotorGroup intake({6,7}, pros::MotorGears::blue);

pros::Motor ladyBrown(-10);
pros::Motor remy(9);

//rotation 1

pros::ADIDigitalOut clamp('A');


// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              10.8, // 11.3 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              480, // drivetrain rpm is 480
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(11);
// horizontal tracking wheel encoder


// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(5.5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(1, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              4, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);



/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */





/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    chassis.calibrate(); // calibrate sensors

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */







void autonomous() {
    chassis.setPose(13,46,-59.03624);
    remy.move_voltage(-1000);

    ladyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    chassis.moveToPoint(48,25,2000, {.forwards = false, .maxSpeed = 40});   //Move to goal
    chassis.waitUntilDone();
    clamp.set_value(true);
    pros::delay(700);
    chassis.turnToPoint(72,24,2000, {.maxSpeed = 40});    //Turn to ring
    chassis.waitUntilDone();
    intake.move_voltage(9000);
    pros::delay(1000);
    chassis.moveToPoint(78,24,2000, {.maxSpeed = 40});    //Ring
    //chassis.moveToPoint(72,24,2000, {.forwards = false, .maxSpeed = 40});    //back up from ring
        intake.move_voltage(7000);

    chassis.waitUntilDone();
    pros::delay(2000);


    //chassis.moveToPoint(94, 20, 2000, {.maxSpeed = 40});    //reposition for corner

    chassis.turnToHeading(-70, 2000, {.maxSpeed = 40}); //turn to corner    
        chassis.moveToPoint(80,19, 2000, {.forwards = false, .maxSpeed = 60}); //Reposition

    chassis.moveToPoint(125,19, 2000, {.forwards = false, .maxSpeed = 60}); //Drop in corner
    chassis.waitUntilDone();
    intake.move_voltage(0);
    clamp.set_value(false);
    pros::delay(700);
    chassis.turnToHeading(0, 2000);

    chassis.moveToPoint(120, 70, 2000, {.maxSpeed = 40});   //reposition

    chassis.turnToHeading(83, 2000, {.maxSpeed = 40}); //turn to goal
    chassis.moveToPoint(98, 55, 2000, {.forwards = false, .maxSpeed = 40});    //Move to goal



    chassis.waitUntilDone();
    clamp.set_value(true);
    pros::delay(700);
    intake.move_voltage(7000);

            chassis.moveToPoint(72, 26, 2000, {.forwards = false, .maxSpeed = 40});    //Reposititon

    chassis.turnToPoint(46,48,2000, {.maxSpeed = 40});  //Turn to ring
    chassis.moveToPoint(46, 48, 2000, {.maxSpeed = 40});    //Move to ring

    chassis.turnToPoint(25,25,2000, {.maxSpeed = 40});


    chassis.moveToPoint(25, 25, 2000, {.maxSpeed = 40});    //Last ring
    ladyBrown.move_voltage(10000);
    chassis.turnToHeading(0,2000);
    chassis.moveToPoint(12,8,2000,{.forwards = false, .maxSpeed = 40});
    chassis.waitUntilDone();
    ladyBrown.move_voltage(0);
    clamp.set_value(false);
    pros::delay(2000);

        chassis.moveToPoint(20,20,2000,{.forwards = false, .maxSpeed = 40});


}



/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void setIntake() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
		intake.move_voltage(-12000);
	}
	else {
		intake.move_voltage(10000);
	}
}

void setClamp() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
		clamp.set_value(false);
	}
	else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
		clamp.set_value(true);
	}
}

void setRemy() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
		remy.move_voltage(7000);
	}
	else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
		remy.move_voltage(-7000);
	}
    else {
        remy.move_voltage(-1200);
    }
}

void setlb() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
		ladyBrown.move_voltage(12000);
	}
	else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
		ladyBrown.move_voltage(-10000);
	}
    else {
        ladyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        ladyBrown.move_velocity(0);

    }
}

void opcontrol() {
    while (true) {
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        chassis.tank(leftY, rightY);

		setIntake();
		setClamp();
        setRemy();
        setlb();

        // delay to save resources
        pros::delay(10);
    }
}

