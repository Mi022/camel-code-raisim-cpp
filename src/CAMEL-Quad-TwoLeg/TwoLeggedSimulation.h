#ifndef RAISIM_TWOLEGGEDSIMULATION_H
#define RAISIM_TWOLEGGEDSIMULATION_H

#include "include/CAMEL/Simulation.h"
#include "TwoLeggedRobot.h"
#include "TwoLeggedPDController.h"

class TwoLeggedSimulation : public Simulation {

public:
    TwoLeggedSimulation(raisim::World *world, double dT) : Simulation(world, dT) { ; }

private:

};


#endif //RAISIM_TWOLEGGEDSIMULATION_H
