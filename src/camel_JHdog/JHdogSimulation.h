//
// Created by jaehyeong on 22. 8. 11.
//

#ifndef RAISIM_JHDOGSIMULATION_H
#define RAISIM_JHDOGSIMULATION_H

#include "include/CAMEL/Simulation.h"
#include "UI/mainwindow.h"
#include "include/RT/rb_utils.h"
#include <QApplication>

class JHdogSimulation : public Simulation {
public:
    //initialize
    JHdogSimulation(raisim::World *world, double dT) : Simulation(world, dT) { ; }

private:
};

#endif //RAISIM_JHDOGSIMULATION_H
