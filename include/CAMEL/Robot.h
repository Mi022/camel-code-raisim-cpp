//
// Created by jaehoon on 22. 3. 31..
//

#ifndef RAISIM_ROBOT_H
#define RAISIM_ROBOT_H

#include"raisim/World.hpp"

class Robot {
public:
    raisim::ArticulatedSystem *robot;
    raisim::World *robotWorld;

    Robot(raisim::World *world, std::string urdfPath, std::string name) {
        robotWorld = world;
        robot = world->addArticulatedSystem(urdfPath);
        robot->setName(name);
    }

    virtual void initialize() = 0;
    raisim::VecDyn getQ(){return robot->getGeneralizedCoordinate();}
    raisim::VecDyn getQD(){return robot->getGeneralizedVelocity();}
    double getWorldTime() { return robotWorld->getWorldTime(); }

private:

};


#endif //RAISIM_ROBOT_H