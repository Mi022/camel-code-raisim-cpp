//
// Created by hwayoung on 22. 8. 17.
//

#ifndef RAISIM_SINGLELEGSIMULATION_H
#define RAISIM_SINGLELEGSIMULATION_H


#include "include/CAMEL/Simulation.h"   //include basic sources
//#include "UI/simulationMainwindow.h"
#include "UI_RealTimePlot/operationMainwindow.h"
#include "include/RT/rb_utils.h"
#include <QApplication>


class SingleLegSimulation : public Simulation {

public:
    SingleLegSimulation(raisim::World *world, double dT) : Simulation(world, dT) {;}
};


#endif //RAISIM_SINGLELEGSIMULATION_H
