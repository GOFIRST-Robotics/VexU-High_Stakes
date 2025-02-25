
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

void rushAuto() {
    remy.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    chassis.setPose(16, 44.5, 112);

    chassis.moveToPoint(56.5, 32, 2000);
    chassis.turnToHeading(170, 2000);
    //chassis.swingToPoint(50, 35, DriveSide::RIGHT, 1000, {.forwards = false});


    chassis.moveToPoint(44, 33, 2000, {.forwards = false});
    chassis.waitUntilDone();
    remy.move_voltage(10000);

    chassis.turnToPoint(56, 40, 2000);
    remy.move_voltage(-7000);

    chassis.moveToPoint(56, 40, 2000, {.maxSpeed = 50});
    chassis.waitUntilDone();
    ladyBrown.move_voltage(10000);
    pros::delay(2000);
    ladyBrown.move_voltage(1000);
}

void goalAutoBlue() {
    chassis.setPose(21,60,-170);
    //ladyBrown.move_voltage(6000);
    chassis.moveToPoint(22,65,2000, {.forwards = false});   //Move to grab goal
    chassis.waitUntilDone();
    clamp.set_value(true);
    pros::delay(2000);
    intake.move_voltage(8000);
    pros::delay(2000);
    intake.move_voltage(0);
    ladyBrown.move_voltage(-10000);
    //clamp.set_value(false);
    pros::delay(2000);
    remy.move_voltage(-3000);
ladyBrown.move_voltage(-1000);


    chassis.moveToPoint(28,49, 2000, {.maxSpeed = 70}); //Reposition to align with bar
    chassis.waitUntilDone();
    ladyBrown.move_voltage(0);
    chassis.moveToPoint(36,36, 2000, {.maxSpeed = 60}); //Move to bar
    chassis.turnToPoint(55,55, 2000, {.maxSpeed = 60});

    chassis.moveToPoint(55,55, 2000, {.maxSpeed = 60}); //Move to touch bar
    chassis.waitUntilDone();
    ladyBrown.move_voltage(10000);
    pros::delay(2000);
    ladyBrown.move_voltage(1000);
}

void goalAutoRed() {
    chassis.setPose(-21,60,170);
    //ladyBrown.move_voltage(6000);
    chassis.moveToPoint(-26,75,2000, {.forwards = false, .maxSpeed = 60});  //Move to grab goal
    chassis.waitUntilDone();
    clamp.set_value(true);
    pros::delay(2000);
    intake.move_voltage(8000);
    pros::delay(2000);
    intake.move_voltage(0);
    ladyBrown.move_voltage(-10000);
    //clamp.set_value(false);
    pros::delay(2000);
    remy.move_voltage(-3000);
ladyBrown.move_voltage(-1000);


    chassis.moveToPoint(-28,49, 2000, {.maxSpeed = 70});    //Reposition to align with bar
    chassis.waitUntilDone();
    ladyBrown.move_voltage(0);
    chassis.moveToPoint(-36,36, 2000, {.maxSpeed = 60});    //Move to bar
    chassis.turnToPoint(-55,55, 2000, {.maxSpeed = 60});

    chassis.moveToPoint(-55,55, 2000, {.maxSpeed = 60});    //Move to touch bar
    chassis.waitUntilDone();
    ladyBrown.move_voltage(10000);
    pros::delay(2000);
    ladyBrown.move_voltage(1000);
}

void autonomous() {
    goalAutoBlue();
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
	else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
		intake.move_voltage(10000);
	}
    else {
        intake.move_voltage(0);
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

