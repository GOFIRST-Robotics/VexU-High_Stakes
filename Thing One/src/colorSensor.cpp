#include "colorSensor.h"

ColorSensor::ColorSensor(char red_port, char blue_port, bool opps) {
    red_sense = new pros::adi::AnalogIn (red_port);
    blue_sense = new pros::adi::AnalogIn (blue_port);

    // true = red, false = blue
    OPP_COLOR = opps;
}

bool ColorSensor::sees_opps() {
    if (OPP_COLOR == TC_RED) {
        return sees_red();
    }
    else {
        return sees_blue();
    }
}

bool ColorSensor::sees_red() {
    return (red_sense->get_value() > 3000);
}

bool ColorSensor::sees_blue() {
    return (blue_sense->get_value() > 3000);
}

void ColorSensor::set_opps(bool opps) {
    OPP_COLOR = opps;
}

bool ColorSensor::get_opps() {
    return OPP_COLOR;
}