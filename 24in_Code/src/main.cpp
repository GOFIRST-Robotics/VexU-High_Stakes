#include "main.h"
#include "lemlib/api.hpp"
//#include "robodash/api.h"



pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::adi::Pneumatics clamp('e', false);
pros::adi::Pneumatics rushMech('f', false);

// left motor group
pros::MotorGroup left_motor_group({1, 21, -13}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({-8, 14, -17}, pros::MotorGears::blue);

pros::MotorGroup intake({3,2}, pros::MotorGears::blue);

//6 lb


// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              11.3, // 11.3 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              480, // drivetrain rpm is 480
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(4);

//rotation 10

// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
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
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    chassis.calibrate();
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
    chassis.setPose(26,34,90);

    rushMech.set_value(true);
    clamp.set_value(false);

    chassis.moveToPoint(60,32, 2000);  //Rush

    rushMech.set_value(false);
    chassis.moveToPoint(40, 32, 35000, {.forwards = false});    //Pull back

    chassis.waitUntilDone();

    rushMech.set_value(true);
    pros::delay(500);
    clamp.set_value(false);

    chassis.turnToHeading(-80, 2000); 

    chassis.moveToPoint(55, 30, 2000, {.forwards = false, .maxSpeed = 60}); //Move to goal
    rushMech.set_value(false);

    chassis.waitUntilDone();
    clamp.set_value(true);  //Grab goal
    intake.move_voltage(8000);       //Scoring preload
    pros::delay(2000);
    intake.move_voltage(0);
    clamp.set_value(false); //Drop goal

    chassis.moveToPoint(25, 50, 2000);  //Go to line up position
    chassis.turnToHeading(180,2000);

    chassis.moveToPoint(25,60,2000,{.forwards = false, .maxSpeed = 70});    //Go to free goal
    chassis.waitUntilDone();

    clamp.set_value(true);  //Pick up goal
    pros::delay(700);

    intake.move_voltage(8000);

    chassis.moveToPoint(25, 25, 2000);  //Ring stack

    /*
    if we get color sort on 24in

    chassis.turnToPoint(90, 2000);
    chassis.moveToPoint(58,25,2000)

    */

    chassis.turnToHeading(45,2000);

    chassis.moveToPoint(52,55,2000);    //Go to touch bar
    intake.move_voltage(0);

    chassis.waitUntilDone();

    //lb movement

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

void opcontrol() {

}