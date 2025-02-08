#pragma once
#include "definitions.h"
#include "controller.h"

class climb {
    public:
        static void init();
        //This is static
        climb() = delete;

        static void periodic();
    
    private:
        static pros::Motor *climbMotor;
        static controller* robotController;
};