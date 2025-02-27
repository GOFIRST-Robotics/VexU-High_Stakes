#pragma once
#include "definitions.h"
//#include "main.h"




class robotDrive {

    public:

        void showEncoderDetails();

        void teleopRoutine();

        void init(); //<- required to get around incompatibility of pros tasks with 
        // object oriented programming.


        // create IMUs
        pros::Imu imu{IMU_PORT_ONE};

        //Create rotaton sensors
        pros::Rotation hPod{HORIZONTAL_WHEEL};
        pros::Rotation vPod{VERTICAL_WHEEL};

        //Instantiate tracking wheels
        lemlib::TrackingWheel horizontalTrackingWheel{&hPod, WHEEL_DIAMETER, HORIZONTAL_WHEEL_OFFSET};
        lemlib::TrackingWheel verticalTrackingWheel{&vPod, WHEEL_DIAMETER, VERTICAL_WHEEL_OFFSET};


        //Instantiate odometry tracking
        lemlib::OdomSensors sensors{nullptr, // Vertical tracking wheel 1, set to null
            nullptr, //vertical tracking wheel 2 which is not used with imu
            nullptr, // horizontal tracking wheel 1
            nullptr, // horizontal tracking wheel 2 which is not used with imu
            &imu
        };

        //Create the drivetrain object
        lemlib::Drivetrain drivetrain{&leftMotors, &rightMotors, TRACK_WIDTH, WHEEL_DIAMETER, DRIVETRAIN_RPM, HORIZONTAL_DRIFT};




        // PID config (THIS WAS DEFAULT COPY/PASTED FROM LEMLIB TUTORIALS WILL BE ADJUSTED LATER)
        // lateral PID controller
        lemlib::ControllerSettings lateral_controller{10, // proportional gain (kP)
                                                    0, // integral gain (kI)
                                                    3, // derivative gain (kD)
                                                    3, // anti windup
                                                    1, // small error range, in inches
                                                    100, // small error range timeout, in milliseconds
                                                    3, // large error range, in inches
                                                    500, // large error range timeout, in milliseconds
                                                    20 // maximum acceleration (slew)
        };

        // angular PID controller
        lemlib::ControllerSettings angular_controller{2, // proportional gain (kP)
                                                    0, // integral gain (kI)
                                                    10, // derivative gain (kD)
                                                    3, // anti windup
                                                    1, // small error range, in degrees
                                                    100, // small error range timeout, in milliseconds
                                                    3, // large error range, in degrees
                                                    500, // large error range timeout, in milliseconds
                                                    0 // maximum acceleration (slew)
        };


        // input curve for throttle input during driver control
        lemlib::ExpoDriveCurve throttle_curve{3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
        };

        // input curve for steer input during driver control
        lemlib::ExpoDriveCurve steer_curve{3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
        };



        //create the chassis
        lemlib::Chassis chassis{
            drivetrain, //drivetrain settings
            lateral_controller, // Lateral PID settings
            angular_controller, // Angular PID settings
            sensors // odometry sensors
        };







    private:
    //Define motor groups with gear ratios
        pros::MotorGroup leftMotors{{FRONT_LEFT_PORT * FRONT_LEFT_DIRECTION , MID_LEFT_PORT * MID_LEFT_DIRECTION, BACK_LEFT_PORT * BACK_LEFT_DIRECTION}, pros::MotorGearset::blue};/*Blue gearset means the motor spins at a max of 600 rpm*/
        pros::MotorGroup rightMotors{{FRONT_RIGHT_PORT * FRONT_RIGHT_DIRECTION, MID_RIGHT_PORT * MID_RIGHT_DIRECTION, BACK_RIGHT_PORT * BACK_RIGHT_PORT}, pros::MotorGearset::blue};








};