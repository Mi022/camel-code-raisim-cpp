//
// Created by hwayoung on 22. 8. 29.
//

#ifndef RAISIM_DOUBLEBARSIMULATION_H
#define RAISIM_DOUBLEBARSIMULATION_H

#include "include/CAMEL/Simulation.h"   //include basic sources
//#include "UI/simulationMainwindow.h"
#include "UI_RealTimePlot/operationMainwindow.h"
#include "include/RT/rb_utils.h"
#include <QApplication>


class DoubleBarSimulation : public Simulation {

public:
    DoubleBarSimulation(raisim::World *world, double dT) : Simulation(world, dT) {;}
};


#endif //RAISIM_DOUBLEBARSIMULATION_H
