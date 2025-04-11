#pragma once
#include "definitions.h"
#define RED true
#define BLUE false

class ColorSensor {

    bool OPP_COLOR = false; // false = blue, true = red

    pros::adi::DigitalIn *red_sense;
    pros::adi::DigitalIn *blue_sense;
    

public:

    ColorSensor(char red_port, char blue_port, bool opps);

    bool sees_opps();

    bool sees_red();

    bool sees_blue();

    void set_opps(bool opps);

    bool get_opps();

};