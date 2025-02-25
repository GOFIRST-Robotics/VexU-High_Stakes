
//This is code for the 15" Robot, for the 24" go to Thing One

#include "main.h"
#include "lemlib/api.hpp"
#include "cmath"
//#include "../include/definitons.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

const float DEGREE_TO_CENTIDEGREE = 100;


// left motor group
pros::MotorGroup left_motor_group({-16, -14, 15}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({18, -19, 20}, pros::MotorGears::blue);

pros::MotorGroup intake({6,7}, pros::MotorGears::blue);

//LadyBrown
pros::Motor ladyBrown(-10);
pros::Rotation ladyJointSensor(11);
const int LADY_BROWN_START_ANGLE = -100 * DEGREE_TO_CENTIDEGREE; // 0 is straight up
const int LADY_BROWN_MAX_ANGLE = 160 * DEGREE_TO_CENTIDEGREE;
const int LADY_BROWN_UP_ANGLE = -30 * DEGREE_TO_CENTIDEGREE;

pros::Motor remy(9);

//rotation 1

pros::ADIDigitalOut clamp('A');


// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              11.3, // 11.3 inch track width
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

    ladyJointSensor.reset_position();
    ladyJointSensor.set_position(LADY_BROWN_START_ANGLE);

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


// ----------------- Intake stuffs -----------------
void in_take() {
    intake.move_voltage(10000);
}

void out_take() {
    intake.move_voltage(-12000);
}

void stop_take() {
    intake.move_voltage(0);
}


// ----------------- Clamp stuffs -----------------
int clamp_count = 0;
bool clamped = false;

void clamp_down() {
    clamp.set_value(true);
}

void clamp_up() {
    clamp.set_value(false);
}


// ----------------- Remy stuffs -----------------
void remi_down() {
    remy.move_voltage(7000);
}

void remi_up() {
    remy.move_voltage(-7000);
}

void remi_stop() {
    remy.move_voltage(-1200);
}


// ----------------- Lady Brown stuffs -----------------
int lady_state = 0;
#define LADY_START 0
#define LADY_OUT 1
#define LADY_IN 2
#define LADY_UP 3
#define LADY_JOY 4

#define LADY_JOY_DEADZONE 70

void lady_out() {
    ladyBrown.move_voltage(10000);
}
void lady_out_slow() {
    ladyBrown.move_voltage(5000);
}

void lady_in() {
    ladyBrown.move_voltage(-8000);
}
void lady_in_slow() {
    ladyBrown.move_voltage(-5000);
}

void lady_up() {
    // TODO
}

void lady_PF_move() {

}

void lady_stop() {
    ladyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    ladyBrown.move_velocity(0);
}


// ----------------- teleop -----------------
void opcontrol() {
    while (true) {

        // ----------------- Drive stuffs -----------------
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        float mult = (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) ? 0.5 : 1.0;
        chassis.arcade(leftY * mult, rightX * mult);


        // ----------------- Intake stuffs -----------------
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            in_take();
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            out_take();
        }
        else {
            stop_take();
        }


        // ----------------- Clamp stuffs -----------------
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) & clamp_count > 50) {
            if (!clamped) {
                clamp_down();
                clamped = true;
            }
            else {
                clamp_up();
                clamped = false;
            }

            clamp_count = 0;
        }
        else {
            clamp_count++;
        }

        
        // ----------------- Remy stuffs -----------------
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            remi_down();
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            remi_up();
        }
        else { 
            remi_stop();
        }


        // ----------------- Lady Brown stuffs -----------------
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        switch(lady_state) {
            case LADY_START:
                lady_state = LADY_IN;
                break;

            case LADY_UP:
                // TODO: PIDF to bring ladybrown up

                lady_stop();

                // Check for moves to other states
                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
                    lady_state = LADY_OUT;
                }
                else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                    lady_state = LADY_IN;
                }
                else if (abs(rightY) > LADY_JOY_DEADZONE) {
                    lady_state = LADY_JOY;
                }

                break;

            case LADY_OUT:
                
                lady_out();

                // Check for moves to other states
                if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
                    lady_state = LADY_UP;
                }

                break;

            case LADY_IN:

                // Move if button is held
                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
                    lady_in();
                }
                else {
                    lady_stop();
                }

                // Check for moves to other states
                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
                    lady_state = LADY_OUT;
                }
                else if (abs(rightY) > LADY_JOY_DEADZONE) {
                    lady_state = LADY_JOY;
                }

                break;
            
            case LADY_JOY:

                // Move based on Joy Inputs
                if (rightY > LADY_JOY_DEADZONE) {
                    lady_out_slow();
                }
                else if (rightY < -LADY_JOY_DEADZONE) {
                    lady_in_slow();
                }
                else {
                    lady_stop();
                }

                // Check for moves to other states
                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
                    lady_state = LADY_OUT;
                }
                else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                    lady_state = LADY_IN;
                }
                break;
        }
        


        // delay to save resources
        pros::delay(10);
    }
}