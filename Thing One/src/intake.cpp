#include "intake.h"

void Intake::initialize_intake(pros::MotorGroup lower_intake_stage, pros::MotorGroup upper_intake_stage, colorSensor colorSensor) {
    lower_stage = &lower_intage_stage;
    upper_stage = &upper_intake_stage;

    color_sensor = &colorSensor;
}

void Intake::initialize_intake(pros::MotorGroup lower_intake_stage, pros::MotorGroup upper_intake_stage) {
    lower_stage = &lower_intage_stage;
    upper_stage = &upper_intake_stage;
}

/*
* Intakes with first stage like normal, but upper stage will try to spit out rings of the wrong color based on the color sensor.
* Call it just like you would the normal intake function to use.
*/
void Intake::filtered_intake() {

    // Run first stage like normal
    move_lower(intake_vel[0]);
    
    switch (intake_state) {
        case INTAKE_CLEAR:
            move_upper(intake_vel[0]);

            // Change state if the wrong color is detected
            if (colorSensor.sees_opps()) {
                intake_state = INTAKE_SPITTING;
            }
            break;
        case INTAKE_SPITTING:
            move_upper(SPITTING_VEL);

            intake_state_machine_timer++;

            // Switch back when the timer runs out
            if(intake_state_machine_timer >= SPIT_TIME) {
                intake_state = INTAKE_CLEAR;
                intake_state_machine_timer = 0;
            }
            break;
    }
    
}

void Intake::intake() {
    upper_stage.move(intake_vel[0]);
    lower_stage.move(intake_vel[1]);
}
void Intake::outtake() {
    upper_stage.move(outtake_vel[0]);
    lower_stage.move(outtake_vel[1]);
}

void Intake::move_upper(int voltage) {
    lower_stage.move(voltage);
}
void Intake::move_lower(int voltage) {
    lower_stage.move(voltage);
}