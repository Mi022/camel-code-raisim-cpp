//
// Created by user on 22. 6. 18.
//

#ifndef RAISIM_MIPISIMULATION_H
#define RAISIM_MIPISIMULATION_H

#include "include/CAMEL/Simulation.h"
#include "MIPIRobot.h"
#include "MIPILQRController.h"

class MIPISimulation : public Simulation{

public:
    MIPISimulation(raisim::World *world, double dT) : Simulation(world, dT) { ; }

private:
};


#endif //RAISIM_MIPISIMULATION_H
