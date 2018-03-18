#include "twist_controller.h"

namespace DBWNODE_NS {

    Controller::Controller() {
        // no-op
    }

    PredictedControlValues Controller::control() {
        float throttle = 12;
        float brake = 0;
        float steer = 0;
        return PredictedControlValues(throttle, brake, steer);
    }

    Controller::~Controller() {
        // no-op
    }
}
