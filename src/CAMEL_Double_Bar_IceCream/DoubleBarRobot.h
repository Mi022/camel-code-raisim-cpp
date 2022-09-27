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
    static const double getDeg2Rad();
    int getDim() const;

protected:
    static const double deg2rad;
    int dim;

private:
};

#endif //RAISIM_DOUBLEBARROBOT_H
