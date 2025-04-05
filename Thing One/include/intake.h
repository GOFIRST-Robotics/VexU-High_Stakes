#pragma once
#include 

class Intake {

    // 1 is first stage, 2 is second stage
    int intake_vel[2];
    int outtake_vel[2];

    pros::MotorGroup *upper_stage;
    pros::MotorGroup *lower_stage;

    colorSensor *color_sensor;

    #define INTAKE_CLEAR 0
    #define INTAKE_SPITTING 1

    int intake_state = INTAKE_CLEAR;
    int intake_state_machine_counter = 0;

    #define SPIT_TIMER 20

public:

    void initialize_intake(pros::MotorGroup lower_stage, pros::MotorGroup upper_stage, colorSensor color_sensor);
    void initialize_intake(pros::MotorGroup lower_stage, pros::MotorGroup upper_stage);

    void filtered_intake();

    void intake();
    void outtake();

    void move_upper(int voltage);
    void move_lower(int voltage);


}