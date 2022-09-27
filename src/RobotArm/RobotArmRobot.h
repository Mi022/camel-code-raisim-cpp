//
// Created by jy on 22. 4. 3..
//

#ifndef RAISIM_SIMPLEPENDULUMROBOT_H
#define RAISIM_SIMPLEPENDULUMROBOT_H

#include "include/CAMEL/Robot.h"
#include "src/RobotArm/RobotArmMotionPlanning.h"

class RobotArmRobot : public Robot
{
public:

    RobotArmRobot(raisim::World* world, std::string urdfPath, std::string name)
        : Robot(world, urdfPath, name)
    {
        if (name == "robotArm")
        {
            initialize();
        }
    }

    void initialize() override;

    raisim::VecDyn getQ();
    raisim::VecDyn getQD();
};


#endif //RAISIM_SIMPLEPENDULUMROBOT_H
