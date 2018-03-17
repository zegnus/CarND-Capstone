#include "PIDControl.h"

namespace DBWNODE_NS {

    //using namespace std;

    PIDControl::PIDControl() {
    }

    PIDControl::PIDControl(const double kp, const double ki, const double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    PIDControl::~PIDControl() {
        // no-op
    }
}
