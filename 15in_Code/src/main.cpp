#include "main.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/llemu.hpp"
#include "pros/screen.hpp"
#include "pros/serial.h"
#include "pros/serial.hpp"
#include <memory>

#define BAUDRATE 115200;

pros::Serial serial(13);


// // drivetrain settings
// lemlib::Drivetrain
//     drivetrain(&left_motors,               // left motor group
//                &right_motors,              // right motor group
//                10,                         // 10 inch track width
//                lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
//                360,                        // drivetrain rpm is 360
//                2                           // horizontal drift is 2 (for now)
//     );

// pros::Imu imu_sensor(10);
// lemlib::OdomSensors sensors(
//     nullptr, // vertical tracking wheel 1, set to null
//     nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
//     nullptr, // horizontal tracking wheel 1
//     nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a
//              // second one
//     &imu_sensor // inertial sensor
// );

// // lateral PID controller
// lemlib::ControllerSettings
//     lateral_controller(10,  // proportional gain (kP)
//                        0,   // integral gain (kI)
//                        3,   // derivative gain (kD)
//                        3,   // anti windup
//                        1,   // small error range, in inches
//                        100, // small error range timeout, in milliseconds
//                        3,   // large error range, in inches
//                        500, // large error range timeout, in milliseconds
//                        20   // maximum acceleration (slew)
//     );

// // angular PID controller
// lemlib::ControllerSettings
//     angular_controller(2,   // proportional gain (kP)
//                        0,   // integral gain (kI)
//                        10,  // derivative gain (kD)
//                        3,   // anti windup
//                        1,   // small error range, in degrees
//                        100, // small error range timeout, in milliseconds
//                        3,   // large error range, in degrees
//                        500, // large error range timeout, in milliseconds
//                        0    // maximum acceleration (slew)
//     );

// // create the chassis
// lemlib::Chassis chassis(drivetrain,         // drivetrain settings
//                         lateral_controller, // lateral PID settings
//                         angular_controller, // angular PID settings
//                         sensors             // odometry sensors

// );

// Tank drive
pros::Controller controller(pros::E_CONTROLLER_MASTER);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    serial.set_baudrate(115200);

    pros::lcd::initialize();
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
void autonomous() {}


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
    // loop forever

    char byte = 'a';
    
    pros::lcd::print(1, "Recieved byte: %c", byte);

    // delay to save resources
    pros::delay(25);
  
}