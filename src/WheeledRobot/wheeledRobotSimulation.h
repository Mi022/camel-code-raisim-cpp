//
// Created by ljm on 22. 7. 6.
//

#ifndef RAISIM_WHEELEDROBOTSIMULATION_H
#define RAISIM_WHEELEDROBOTSIMULATION_H

#include "include/CAMEL/Simulation.h"
#include "wheeledRobot.h"
#include "wheeledRobotController.h"

class wheeledRobotSimulation:public Simulation{
public:
    wheeledRobotSimulation(raisim::World *world, double dT) : Simulation(world, dT) { ; }

private:
};

#endif //RAISIM_WHEELEDROBOTSIMULATION_H
