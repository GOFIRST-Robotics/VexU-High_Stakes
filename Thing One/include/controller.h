#pragma once
#include "definitions.h"


class controller : public pros::Controller {
    private:
        static controller* instance;
        controller();
        using pros::Controller::Controller;
    public:
        
        static controller* getInstance();

};