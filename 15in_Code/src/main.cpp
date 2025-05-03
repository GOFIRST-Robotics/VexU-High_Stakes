#include "main.h"
#include "lemlib/api.hpp"
#include "colorSensor.h"
#include "intake.h"
//#include "robodash/api.h"


/*
rd::Selector selector({
    {"SKILLS", skills},
    {"BLUE SIDE GOAL", blueSideGoal},
    {"BLUE MID GOAL", blueMidGoal},
    {"RED SIDE GOAL", redSideGoal},
    {"RED MID GOAL", redMidGoal},
});

*/
//rd::Console console;


pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::adi::Encoder horizontal_encoder('C', 'D');
pros::adi::Encoder vertical_encoder('E', 'F');

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, .177, 0);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, .177, 0);

pros::adi::Pneumatics clamp('G', false);
pros::adi::Pneumatics rushMech('H', false);

// left drive motor group
pros::MotorGroup left_motor_group({18, -7, -5}, pros::MotorGears::blue);
// right drive motor group
pros::MotorGroup right_motor_group({-2, 12, 3}, pros::MotorGears::blue);

pros::MotorGroup intake_first_stage_motor_group({4}, pros::MotorGears::blue);
pros::MotorGroup intake_second_stage_motor_group({8}, pros::MotorGears::blue);

//LadyBrown
pros::Motor ladyBrown(1);
pros::Rotation ladyJointSensor(10);
const int LADY_BROWN_MAX_ANGLE = 355;
const int LADY_BROWN_MIN_ANGLE = 192;
const int LADY_BROWN_COLLECT_ANGLE = 220;
const int LADY_BROWN_START_ANGLE = LADY_BROWN_MIN_ANGLE; // 100 is all the way back
const int LADY_BROWN_SCORE_ANGLE = 320;
const int LADY_BROWN_UP_ANGLE = 295;

ColorSensor *intake_color_sensor;
Intake *intake;

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              11.3, // 11.3 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              600, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(11);


// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
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
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();    //Kept in case of debugging
    pros::screen::set_pen(pros::c::COLOR_BLUE);

    chassis.calibrate();

    ladyJointSensor.reset_position();
    ladyJointSensor.set_reversed(true);

    intake_color_sensor = new ColorSensor('B', 'A', TC_BLUE);
    intake = new Intake(&intake_first_stage_motor_group, &intake_second_stage_motor_group, intake_color_sensor);
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
    //selector.run_auton();
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


// ----------------- Clamp stuffs -----------------
int clamp_count = 0;
bool clamped = false;

void clamp_down() {
    clamp.set_value(true);
}

void clamp_up() {
    clamp.set_value(false);
}


// ----------------- Rush stuffs -----------------

void rush_down() {
    rushMech.set_value(true);
}

void rush_up() {
    rushMech.set_value(false);
}


// ----------------- Lady Brown stuffs -----------------
#define LADY_START 0
#define LADY_OUT 1
#define LADY_STOW 2
#define LADY_UP 3
#define LADY_COLLECT 4
#define LADY_JOY 5

int lady_state = LADY_START;

int lady_intake_timer = 0;

#define LADY_JOY_DEADZONE 60

int get_lady_angle() {
    // returns in degrees from straight up
    return float(ladyJointSensor.get_angle()) / 100;
}

void lady_PF_move(float targ) {

    if (targ > LADY_BROWN_MAX_ANGLE) {
        targ = LADY_BROWN_MAX_ANGLE;
    }
    else if (targ < LADY_BROWN_MIN_ANGLE) {
        targ = LADY_BROWN_MAX_ANGLE;
    }

    float pos = get_lady_angle();

    float pow = 0;

    float kP = 0.01; // TODO: Tune in kP
    pow += kP * (targ - pos);

    // controller.print(0, 0, "%f, %f", pos, targ - pos);

    float kF = 0.08;
    pow += kF * (sin(((LADY_BROWN_UP_ANGLE - pos) * (3.14159/180))));

    ladyBrown.move_voltage(pow * 12000);
}

void lady_up() {
    lady_PF_move(LADY_BROWN_UP_ANGLE);
}

void lady_out() {
    ladyBrown.move_voltage(10000);
}

void lady_collect() {
    lady_PF_move(LADY_BROWN_COLLECT_ANGLE);
}

bool is_lady_collect() {
    return get_lady_angle() < LADY_BROWN_COLLECT_ANGLE + 10;
}

void lady_move(int percent) {
    ladyBrown.move_voltage(7000 * (percent/100));
}

void lady_stow() {
    lady_PF_move(LADY_BROWN_MIN_ANGLE);
}

void lady_score() {
    lady_PF_move(LADY_BROWN_SCORE_ANGLE);
}

void lady_stop() {
    ladyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    ladyBrown.move_velocity(0);
}




void opcontrol() {

    
    while (true) {

        // ------------------ Tracking ------------------ 

        // print measurements from the rotation sensor
        //pros::lcd::print(1, "Vertical Sensor: %i", .get_position());
        //pros::lcd::print(2, "Horizontal Sensor: %i", hPod.get_position());
        lemlib::Pose pose = chassis.getPose();
        // print the x, y, and theta values of the pose

        pros::lcd::print(1, "X: %f", pose.x);
        pros::lcd::print(2, "Y: %f", pose.y);
        pros::lcd::print(3, "Theta: %f", pose.theta);
        pros::delay(10); // delay to save resources. DO NOT REMOVE
    


        // ----------------- Drive stuffs -----------------
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Slow down 
        float mult = (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) ? 0.5 : 1.0;
        chassis.arcade(leftY * mult, rightX * mult);


        // ----------------- Intake stuffs -----------------
        
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) || (lady_state == LADY_OUT & lady_intake_timer > 20)) {
            intake->outtake();
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) || (lady_state == LADY_COLLECT  & is_lady_collect())) {
            intake->intake();
        }
        else {
            intake->stop();
        }

        lady_intake_timer++;


        // ----------------- Clamp stuffs -----------------
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) & clamp_count > 50) {

            // Toggle between clampeded and not
            if (!clamped) {
                clamp_down();
                clamped = true;
            }
            else {
                clamp_up();
                clamped = false;
            }

            clamp_count = 0; // reset counter
        }
        else {
            clamp_count++;
        }

        
        // ----------------- Rush stuffs -----------------
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            rush_down();
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            rush_up();
        }


        // ----------------- Lady Brown stuffs -----------------
        controller.print(0, 0, "%d, %d", get_lady_angle(), lady_state);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // Lady Brown Finate State Machine
        switch(lady_state) {
            case LADY_START:
                lady_state = LADY_STOW;
                break;

            case LADY_STOW:

                lady_stow();

                // Check for moves to other states
                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
                    lady_state = LADY_OUT;
                }
                else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                    lady_state = LADY_COLLECT;
                }
                else if (abs(rightY) > LADY_JOY_DEADZONE) {
                    lady_state = LADY_JOY;
                }

                break;

            case LADY_OUT:
                
                lady_score();

                // Check for moves to other states
                if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
                    lady_state = LADY_STOW;
                }

                break;

            case LADY_COLLECT:

                lady_collect();

                // Check for moves to other states
                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
                    lady_state = LADY_OUT;
                    lady_intake_timer = 0;
                }
                else if (abs(rightY) > LADY_JOY_DEADZONE) {
                    lady_state = LADY_JOY;
                }

                break;
            
            case LADY_JOY:

                // Move based on Joy Inputs
                if (abs(rightY) > LADY_JOY_DEADZONE) {
                    lady_move(rightY);
                }
                else {
                    lady_stop();
                }

                // Check for moves to other states
                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
                    lady_state = LADY_OUT;
                }
                else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                    lady_state = LADY_COLLECT;
                }
                break;
        }


        // delay to save resources
        pros::delay(10);
    }
}
