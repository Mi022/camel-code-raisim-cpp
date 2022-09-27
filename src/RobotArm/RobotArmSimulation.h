//
// Created by jy on 22. 7. 4.
//

#ifndef RAISIM_ROBOTARMSIMULATION_H
#define RAISIM_ROBOTARMSIMULATION_H

#include "include/CAMEL/Simulation.h"
#include "RobotArmRobot.h"
#include "RobotArmPDController.h"
#include "Eigen/Eigen"

using namespace Eigen;

class RobotArmSimulation : public Simulation
{

public:

    RobotArmSimulation(raisim::World* world, double dT)
        : Simulation(world, dT)
    {

    }

};


#endif //RAISIM_ROBOTARMSIMULATION_H
