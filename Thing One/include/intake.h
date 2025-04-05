#pragma once
#include 

class Intake {

    // 1 is first stage, 2 is second stage
    int intake_vel[2];
    int outtake_vel[2];

    pros::MotorGroup *upper_stage;
    pros::MotorGroup *lower_stage;

    colorSensor *color_sensor;

public:

    void initialize_intake(pros::MotorGroup lower_stage, pros::MotorGroup upper_stage, colorSensor color_sensor);
    void initialize_intake(pros::MotorGroup lower_stage, pros::MotorGroup upper_stage);

    void filtered_intake();

    void intake();
    void outtake();

    void move_upper(int voltage);
    void move_lower(int voltage);


}