//
// Created by jy on 22. 4. 3..
//

#ifndef RAISIM_SIMPLEPENDULUMROBOT_H
#define RAISIM_SIMPLEPENDULUMROBOT_H

#include "include/CAMEL/Robot.h"

class RobotArmRobot : public Robot {
public:
    RobotArmRobot(raisim::World *world, std::string urdfPath, std::string name) : Robot(world, urdfPath, name) {
        initialize();
    }

    void initialize() override;
//    double getQ();
//    double getQD();
    raisim::VecDyn getQ();
    raisim::VecDyn getQD();
};


#endif //RAISIM_SIMPLEPENDULUMROBOT_H
