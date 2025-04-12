#pragma once
#include "definitions.h"
#define TC_RED true // "Team Color" red
#define TC_BLUE false // "Team Color" blue

class ColorSensor {

    bool OPP_COLOR = false; // false = blue, true = red

    pros::adi::AnalogIn *red_sense;
    pros::adi::AnalogIn *blue_sense;
    

public:

    ColorSensor(char red_port, char blue_port, bool opps);

    bool sees_opps();

    bool sees_red();

    bool sees_blue();

    void set_opps(bool opps);

    bool get_opps();

};