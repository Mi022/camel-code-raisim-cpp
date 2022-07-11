//
// Created by jaehoon on 22. 3. 31..
//

#ifndef RAISIM_SIMPLEPENDULUMSIMULATION_H
#define RAISIM_SIMPLEPENDULUMSIMULATION_H

#include "include/CAMEL/Simulation.h"
#include "TwoLeggedRobot.h"
#include "TwoLeggedPDController.h"

class SingleLeggedSimulation : public Simulation {

public:
    SingleLeggedSimulation(raisim::World *world, double dT) : Simulation(world, dT) { ; }

private:

};


#endif //RAISIM_SIMPLEPENDULUMSIMULATION_H
