//
// Created by user on 22. 6. 9.
//

#ifndef RAISIM_MIPSIMULATION_H
#define RAISIM_MIPSIMULATION_H

#include "include/CAMEL/Simulation.h"
#include "MIPRobot.h"
#include "MIPPDController.h"
#include "MIPLQRController.h"

class MIPSimulation : public Simulation{

public:
    MIPSimulation(raisim::World *world, double dT) : Simulation(world, dT) { ; }

private:
};


#endif //RAISIM_MIPSIMULATION_H
