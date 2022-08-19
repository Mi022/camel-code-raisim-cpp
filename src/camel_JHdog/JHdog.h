//
// Created by jaehyeong on 22. 8. 11.
//
#ifndef RAISIM_JHDOG_H
#define RAISIM_JHDOG_H

#include "include/CAMEL/Robot.h"

class JHdog : public Robot {
public:
    //initialize
    JHdog(raisim::World *world, std::string urdfPath, std::string name) : Robot(world, urdfPath, name) {
        initialize();
    }
    void initialize() override;
    //override abstract method in robot
};

#endif //RAISIM_JHDOG_H
