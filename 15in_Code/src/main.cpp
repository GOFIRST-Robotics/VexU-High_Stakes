#include "main.h"
#include "lemlib/api.hpp"
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

// left motor group
pros::MotorGroup left_motor_group({1, -20, -16}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({-8, 11, 12}, pros::MotorGears::blue);

pros::MotorGroup intake({-9,10}, pros::MotorGears::blue);


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

    
    while (true) { // infinite loop
        // print measurements from the rotation sensor
        //pros::lcd::print(1, "Vertical Sensor: %i", .get_position());
        //pros::lcd::print(2, "Horizontal Sensor: %i", hPod.get_position());
        lemlib::Pose pose = chassis.getPose();
// print the x, y, and theta values of the pose

        pros::lcd::print(1, "X: %f", pose.x);
        pros::lcd::print(2, "Y: %f", pose.y);
        pros::lcd::print(3, "Theta: %f", pose.theta);
        pros::delay(10); // delay to save resources. DO NOT REMOVE
    }

    while (true) {
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        //chassis.tank(leftY, rightY);
        chassis.tank(leftY, rightY);

        setIntake();
        setClamp1();
        setRushMech1();

        // delay to save resources
        pros::delay(10);
    }
}
