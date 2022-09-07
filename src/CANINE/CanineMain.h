//
// Created by hs on 22. 8. 21.
//

#ifndef RAISIM_CANINEMAIN_H
#define RAISIM_CANINEMAIN_H

#include "include/CAMEL/Simulation.h"
#include "canine/CanineRobot.h"
#include "convexMPC/MPCController.h"

#include "include/RT/rb_utils.h"
#include "GUI/mainwindow.h"
#include "GUI/SharedMemory.h"


#include <cmath>

class CanineMain : public Simulation {
public:
    CanineMain(raisim::World *world, double dT) : Simulation(world, dT) { ; }

private:

};


#endif //RAISIM_CANINEMAIN_H
