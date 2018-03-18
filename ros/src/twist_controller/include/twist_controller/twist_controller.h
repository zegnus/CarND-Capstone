#ifndef TWIST_CONTROLLER__H
#define TWIST_CONTROLLER__H

#include "predicted_control_values.h"

namespace DBWNODE_NS {

    class Controller {
        public:
            Controller();
            PredictedControlValues control();
            ~Controller();

        private:
            // no private members
    };
}

#endif
