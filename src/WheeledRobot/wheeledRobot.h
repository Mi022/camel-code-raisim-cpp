//
// Created by ljm on 22. 7. 6.
//

#ifndef RAISIM_WHEELEDROBOT_H
#define RAISIM_WHEELEDROBOT_H

#include "include/CAMEL/Robot.h"

class wheeledRobot : public Robot {
public:
    wheeledRobot(raisim::World *world, std::string urdfPath, std::string name) : Robot(world, urdfPath, name) {
        initialize();
    }

    void initialize() override;
    raisim::VecDyn getQ();
    raisim::VecDyn getQD();
};

#endif //RAISIM_WHEELEDROBOT_H
