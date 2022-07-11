//
// Created by jaehoon on 22. 4. 3..
//

#ifndef RAISIM_DONGDOGSINGLELEGROBOT_H
#define RAISIM_DONGDOGSINGLELEGROBOT_H

#include "include/CAMEL/Robot.h"

class DongdogsinglelegRobot : public Robot {
public:
    DongdogsinglelegRobot(raisim::World *world, std::string urdfPath, std::string name) : Robot(world, urdfPath, name) {
        initialize();
    }

    void initialize() override;
    double getQ();
    double getQD();
};


#endif //RAISIM_SIMPLEPENDULUMROBOT_H
