#include "intake.h"

void intake::initialize_intake(pros::MotorGroup lower_intake_stage, pros::MotorGroup upper_intake_stage, colorSensor colorSensor) {
    lower_stage = &lower_intage_stage;
    upper_stage = &upper_intake_stage;

    color_sensor = &colorSensor;
}

void intake::initialize_intake(pros::MotorGroup lower_intake_stage, pros::MotorGroup upper_intake_stage) {
    lower_stage = &lower_intage_stage;
    upper_stage = &upper_intake_stage;
}

void intake::filtered_intake() {

    intake();
}

void intake::intake() {
    upper_stage.move(intake_vel[0]);
    lower_stage.move(intake_vel[1]);
}
void intake::outtake() {
    upper_stage.move(outtake_vel[0]);
    lower_stage.move(outtake_vel[1]);
}

void intake::move_upper(int voltage) {
    lower_stage.move(voltage);
}
void intake::move_lower(int voltage) {
    lower_stage.move(voltage);
}