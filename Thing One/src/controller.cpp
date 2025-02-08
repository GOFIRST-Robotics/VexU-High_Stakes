#include "controller.h"

controller* controller::instance = new controller(pros::E_CONTROLLER_MASTER);

controller* controller::getInstance() {
    return instance;
}