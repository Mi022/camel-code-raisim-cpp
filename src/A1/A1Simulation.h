//
// Created by jaehoon on 22. 5. 2.
//

#ifndef RAISIM_A1SIMULATION_H
#define RAISIM_A1SIMULATION_H

#include "include/CAMEL/Simulation.h"
#include "A1Robot.h"
#include "A1JointPDController.h"
#include "A1MPCController.h"
#include "A1JoyStick.h"

class A1Simulation : public Simulation {
public:
    A1Simulation(raisim::World *world, double dT) : Simulation(world, dT) { ; }

private:

};


#endif //RAISIM_A1SIMULATION_H
