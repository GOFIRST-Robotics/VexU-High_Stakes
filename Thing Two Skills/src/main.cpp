
//This is code for the 15" Robot, for the 24" go to Thing One

#include "main.h"
#include "lemlib/api.hpp"
//#include "../include/definitons.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);


// left motor group
pros::MotorGroup left_motor_group({1, -20, -16}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({-8, 11, 12}, pros::MotorGears::blue);

pros::MotorGroup intake({-9,10}, pros::MotorGears::blue);

pros::ADIDigitalOut clamp('D');
pros::ADIDigitalOut rushMech('C');


// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              11.3, // 11.3 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              600, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(13);
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
                                              1, // derivative gain (kD)
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
                                              2, // derivative gain (kD)
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
	chassis.setPose(9.5,72,90);
	intake.move_voltage(9500);
	pros::delay(1000);
	chassis.moveToPoint(20,72,2000);	//Ring in front of robot
	chassis.waitUntilDone();
	intake.move_voltage(0);

	chassis.turnToHeading(-135,800);	//Turn with back to goal

	chassis.moveToPoint(53,121,2000,{.forwards = false, .maxSpeed = 50});	//Back into goal 1
	chassis.waitUntilDone();
	pros::delay(200);
	clamp.set_value(true);	//Pick up goal 1
	pros::delay(700);
	intake.move_voltage(9500);
	pros::delay(1500);

	chassis.moveToPoint(50, 100, 2000, {.maxSpeed = 60});	//Pick up first ring

	chassis.turnToPoint(74,103,2000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
	chassis.moveToPoint(74,115,2000, {.maxSpeed = 60});	//Pick up second ring
	chassis.turnToPoint(74,126,2000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});

	chassis.moveToPoint(74,126,2000);	//Pick up ring stack
	chassis.moveToPoint(74,120,2000, {.forwards = false, .maxSpeed = 60});	//Back away from ring stack to avoid blue ring
	pros::delay(250);
	chassis.waitUntilDone();
	intake.move_voltage(0);

	chassis.moveToPoint(30,120,2000, {.maxSpeed = 70});	//final ring
	pros::delay(50);
	intake.move_voltage(9500);
	chassis.turnToHeading(135,2000);
	chassis.moveToPoint(15,132,1000, {.forwards = false, .maxSpeed = 60});	//Drop goal in corner
	clamp.set_value(false);
	pros::delay(500);

	chassis.moveToPoint(100,100,2000, {.maxSpeed = 60});	//Move to goal 2
	chassis.turnToHeading(-35,2000, {.maxSpeed = 60});	//Turn with back to goal
	chassis.moveToPoint(100,85,2000, {.forwards = false, .maxSpeed = 60});	//Back into goal
	chassis.waitUntilDone();

	clamp.set_value(true);	//Goal 2 grab
	pros::delay(700);

	chassis.moveToPoint(90,116,2000, {.maxSpeed = 60});	//Ring stack 1
	chassis.moveToPoint(90,110,2000, {.forwards = false, .maxSpeed = 60});	//Back away to avoid blue ring


	chassis.turnToPoint(110,121,2000, {.maxSpeed = 60});
	chassis.moveToPoint(110,121,2000, {.maxSpeed = 60});	//Ring stack 2

	chassis.turnToHeading(-135,2000, {.maxSpeed = 60});	//Turn with back to corner
	chassis.moveToPoint(130,135,2000, {.forwards = false, .maxSpeed = 60});	//Drop goal in corner
	chassis.waitUntilDone();
	clamp.set_value(false);
	pros::delay(1000);
	intake.move_voltage(5000);
	chassis.moveToPoint(121,97,2000, {.maxSpeed = 60});	//Ring stack in front of goal 3
	chassis.waitUntilDone();
	pros::delay(100);
	intake.move_voltage(0);

	chassis.turnToHeading(0,2000, {.maxSpeed = 60});	//Goal
	chassis.moveToPoint(121,74,2000, {.forwards = false, .maxSpeed = 60});	//Back into and pick up goal 3
	chassis.waitUntilDone();
	pros::delay(400);
	clamp.set_value(true);
	pros::delay(700);
	intake.move_voltage(9500);
	pros::delay(1000);
	intake.move_voltage(6000);

	chassis.moveToPoint(88,90,2000, {.maxSpeed = 60});	//Reposition to center
	chassis.waitUntilDone();
	intake.move_voltage(9500);

	chassis.moveToPoint(76,73,2000, {.maxSpeed = 60});	//Drive into center
	chassis.moveToPoint(68,76,2000, {.maxSpeed = 60});	//Reposition in center

	chassis.swingToHeading(0, DriveSide::RIGHT, 1000, {.maxSpeed = 60});	//Sweep to pick up rings
	chassis.swingToHeading(170, DriveSide::RIGHT, 1000, {.maxSpeed = 60});	//Sweep to pick up rings
    	chassis.swingToHeading(20, DriveSide::RIGHT, 1000, {.maxSpeed = 60});	//Sweep to pick up rings

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
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        intake.move_voltage(-12000);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        intake.move_voltage(9500);
    }
    else {
        intake.move_voltage(0);
    }
}

bool clampToggle = false;
bool clampLock = false;
void setClamp1() {
    if (clampToggle) {
        clamp.set_value(true);
    } else {
        clamp.set_value(false);
    }
    
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        if (!clampLock) {
            clampToggle = !clampToggle;
            clampLock = true;
        } 
    } else { clampLock = false; }
}

bool rushToggle = false;
bool rushLock = false;
void setRushMech1() {
    if (rushToggle) {
        rushMech.set_value(true);
    } else {
        rushMech.set_value(false);
    }
    
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        if (!rushLock) {
            rushToggle = !rushToggle;
            rushLock = true;
        } 
    } else { rushLock = false; }
}

void opcontrol() {
    while (true) {
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        //chassis.tank(leftY, rightY);
        chassis.arcade(leftY, rightX);

        setIntake();
        setClamp1();
        setRushMech1();

        // delay to save resources
        pros::delay(10);
    }
}

