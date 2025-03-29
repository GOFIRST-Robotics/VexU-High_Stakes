#include "colorSensor.h"

void colorSensor::setup_color(int red_port, int blue_port, bool opps) {
    pros::ADIDigitalIn red_sense (red_port);
    pros::ADIDigitalIn blue_sense (blue_port);

    OPP_COLOR = opps;
}

bool colorSensor::sees_opps() {
    if (OPP_COLOR == RED) {
        return sees_red();
    }
    else {
        return sees_blue();
    }
}

bool colorSensor::sees_red() {
    return red_sense.get_value();
}

bool colorSensor::sees_blue() {
    return blue_sense.get_value();
}

void colorSensor::set_opps(bool opps) {
    OPP_COLOR = opps;
}

bool colorSensor::get_opps() {
    return OPP_COLOR;
}