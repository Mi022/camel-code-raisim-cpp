//
// Created by jaehoon on 22. 4. 3..
//

#ifndef RAISIM_SIMULATION_H
#define RAISIM_SIMULATION_H

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

class Simulation {

public:
    Simulation(raisim::World *world, double dT) {
        this->mWorld = world;
        mWorld->setTimeStep(dT);
        mWorld->addGround();
    }

private:
    raisim::World *mWorld;
};

#endif //RAISIM_SIMULATION_H
