//
// Created by jaehoon on 22. 4. 3..
//

#ifndef RAISIM_SIMPLEPENDULUMROBOT_H
#define RAISIM_SIMPLEPENDULUMROBOT_H

#include "include/CAMEL/Robot.h"

class SimplePendulumRobot : public Robot {
public:
    SimplePendulumRobot(raisim::World *world, std::string urdfPath, std::string name) : Robot(world, urdfPath, name) {
        initialize();
    }

    void initialize() override;
    double getQ();
    double getQD();
};


#endif //RAISIM_SIMPLEPENDULUMROBOT_H
