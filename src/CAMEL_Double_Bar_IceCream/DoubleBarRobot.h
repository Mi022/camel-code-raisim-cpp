//
// Created by hwayoung on 22. 8. 29.
//

#ifndef RAISIM_DOUBLEBARROBOT_H
#define RAISIM_DOUBLEBARROBOT_H

#include "include/CAMEL/Robot.h"

class DoubleBarRobot : public Robot {
public:
    DoubleBarRobot(raisim::World *world, std::string urdfPath, std::string name) : Robot(world, urdfPath, name){
        initialize();
        dim = robot->getGeneralizedCoordinateDim();
    }

    void initialize() override;
    int dim;
    static const double deg2rad;

private:
};

#endif //RAISIM_DOUBLEBARROBOT_H
