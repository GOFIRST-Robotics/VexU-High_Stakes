#pragma once

class Intake {

    int intake_vel[2];
    int outtake_vel[2];

public:

    void initialize_intake(pros::MotorGroup lower_stage, pros::MotorGroup upper_stage);

    void filtered_intake();

    void intake();
    void outtake();

    void move_upper(int voltage);
    void move_lower(int voltage);


}