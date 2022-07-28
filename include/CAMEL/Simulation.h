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
        mGround = mWorld->addGround(0,"gnd");
    }

    void setGroundProperty(std::string groundProperty){mGround->setAppearance(groundProperty);}

private:
    raisim::World *mWorld;
    raisim::Ground *mGround;
};

#endif //RAISIM_SIMULATION_H
