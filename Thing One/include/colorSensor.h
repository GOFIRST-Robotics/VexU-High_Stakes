#pragma once

class ColorSensor {

    const bool RED = true;
    const bool BLUE = false;
    bool OPP_COLOR = false; // false = blue, true = red

public:

    void setup_color(int red_port, int blue_port, bool opps);

    bool sees_opps();

    bool sees_red();

    bool sees_blue();

    void set_opps(bool opps);

    bool get_opps();

}