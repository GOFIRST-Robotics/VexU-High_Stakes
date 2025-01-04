//This is code for the 24" Robot, for the 15" go to Thing Two


/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * \copyright Copyright (c) 2017-2023, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

/**
 * If defined, some commonly used enums will have preprocessor macros which give
 * a shorter, more convenient naming pattern. If this isn't desired, simply
 * comment the following line out.
 *
 * For instance, E_CONTROLLER_MASTER has a shorter name: CONTROLLER_MASTER.
 * E_CONTROLLER_MASTER is pedantically correct within the PROS styleguide, but
 * not convenient for most student programmers.
 */
#define PROS_USE_SIMPLE_NAMES

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
#define PROS_USE_LITERALS
#include "definitions.h"
#include "api.h"

/**
 * You should add more #includes here
 */
//#include "okapi/api.hpp"

/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
// using namespace pros;
// using namespace pros::literals;
// using namespace okapi;



//Define motor groups with gear ratios
pros::MotorGroup leftMotors({FRONT_LEFT_PORT * FRONT_LEFT_DIRECTION , MID_LEFT_PORT * MID_LEFT_DIRECTION, BACK_LEFT_PORT * BACK_LEFT_DIRECTION}, pros::MotorGearset::blue);/*Blue gearset means the motor spins at a max of 600 rpm*/
pros::MotorGroup rightMotors({FRONT_RIGHT_PORT * FRONT_RIGHT_DIRECTION, MID_RIGHT_PORT * MID_RIGHT_DIRECTION, BACK_RIGHT_PORT * BACK_RIGHT_PORT}, pros::MotorGearset::blue);


// create IMUs
pros::Imu imu(IMU_PORT_ONE);

//Create rotaton sensors
pros::Rotation hPod(HORIZONTAL_WHEEL);
pros::Rotation vPod(VERTICAL_WHEEL);

//Instantiate tracking wheels
lemlib::TrackingWheel horizontalTrackingWheel(&hPod, WHEEL_DIAMETER, HORIZONTAL_WHEEL_OFFSET);
lemlib::TrackingWheel verticalTrackingWheel(&vPod, WHEEL_DIAMETER, VERTICAL_WHEEL_OFFSET);


//Instantiate odometry tracking
lemlib::OdomSensors sensors(&verticalTrackingWheel, // Vertical tracking wheel 1, set to null
    nullptr, //vertical tracking wheel 2 which is not used with imu
    &horizontalTrackingWheel, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2 which is not used with imu
    &imu
);

//Create the drivetrain object
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, TRACK_WIDTH, WHEEL_DIAMETER, DRIVETRAIN_RPM, HORIZONTAL_DRIFT);




// PID config (THIS WAS DEFAULT COPY/PASTED FROM LEMLIB TUTORIALS WILL BE ADJUSTED LATER)
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


//create the chassis
lemlib::Chassis chassis(
    drivetrain, //drivetrain settings
    lateral_controller, // Lateral PID settings
    angular_controller, // Angular PID settings
    sensors // odometry sensors
);






void showEncoderDetails() {

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


/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
//#include <iostream>
#endif

#endif  // _PROS_MAIN_H_
