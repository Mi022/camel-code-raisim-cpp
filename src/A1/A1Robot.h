//
// Created by jaehoon on 22. 5. 2.
//

#ifndef RAISIM_A1ROBOT_H
#define RAISIM_A1ROBOT_H

#include "include/CAMEL/Robot.h"

class A1Robot : public Robot {
public:
    A1Robot(raisim::World *world, std::string urdfPath, std::string name) : Robot(world, urdfPath, name) {
        initialize();
    }

    void initialize() override;
};


#endif //RAISIM_A1ROBOT_H
