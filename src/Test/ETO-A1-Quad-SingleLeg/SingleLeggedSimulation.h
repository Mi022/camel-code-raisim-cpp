//
// Created by jaehoon on 22. 3. 31..
//

#ifndef RAISIM_SIMPLEPENDULUMSIMULATION_H
#define RAISIM_SIMPLEPENDULUMSIMULATION_H

#include "include/CAMEL/Simulation.h"
#include "SingleLeggedRobot.h"
#include "SingleLeggedPDController.h"
#include "SingleLeggedIDController.h"
#include "SingleLeggedMPCController.h"
#include "SingleLeggedMPCqpoases.h"
#include "raisim/World.hpp"

class SingleLeggedSimulation : public Simulation {

public:
    SingleLeggedSimulation(raisim::World *world, double dT) : Simulation(world, dT) { ; }

private:

};


#endif //RAISIM_SIMPLEPENDULUMSIMULATION_H
