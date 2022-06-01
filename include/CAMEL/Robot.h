//
// Created by jaehoon on 22. 3. 31..
//

#ifndef RAISIM_ROBOT_H
#define RAISIM_ROBOT_H

#include"raisim/World.hpp"

class Robot {
public:
    raisim::ArticulatedSystem *robot;

    Robot(raisim::World *world, std::string urdfPath, std::string name) {
        robot = world->addArticulatedSystem(urdfPath);
        robot->setName(name);
    }

    virtual void initialize() = 0;

private:

};


#endif //RAISIM_ROBOT_H