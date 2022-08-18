//
// Created by hwayoung on 22. 8. 17.
//

#ifndef RAISIM_SINGLELEGROBOT_H
#define RAISIM_SINGLELEGROBOT_H

#include "include/CAMEL/Robot.h"

class SingleLegRobot : public Robot {
public:
    SingleLegRobot(raisim::World *world, std::string urdfPath, std::string name) : Robot(world, urdfPath, name){
        initialize();
    }

    void initialize() override;

private:
    static const double deg2rad;
};


#endif //RAISIM_SINGLELEGROBOT_H
