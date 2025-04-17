#pragma once
#include "definitions.h"
#include "colorSensor.h"

#define INTAKE_CLEAR 0
#define INTAKE_SPITTING 1

#define SPIT_TIME 20

class Intake {

    // 1 is first stage, 2 is second stage
    int intake_vel[2];
    int outtake_vel[2];

    int spitting_vel;

    pros::MotorGroup *upper_stage;
    pros::MotorGroup *lower_stage;

    ColorSensor *color_sensor;

    int intake_state = INTAKE_CLEAR;
    int intake_state_machine_timer = 0;

public:

    Intake(pros::v5::MotorGroup *lower_stage, pros::v5::MotorGroup *upper_stage, ColorSensor *color_sensor);
    Intake(pros::v5::MotorGroup *lower_stage, pros::v5::MotorGroup *upper_stage);

    void filtered_intake();

    void intake();
    void outtake();

    void move_upper(int voltage);
    void move_lower(int voltage);

};