#pragma once
#include "definitions.h"
#include "controller.h"




class intake {
    public:

        static void init();

        //static
        intake() = delete;

        static void periodic();


    private:
        static pros::Motor* intakeMotor;
        static controller* robotController;




};