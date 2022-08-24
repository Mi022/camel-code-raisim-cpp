
#ifndef RAISIM_TWOLEGGEDROBOT_H
#define RAISIM_TWOLEGGEDROBOT_H

#include "include/CAMEL/Robot.h"

class TwoLeggedRobot : public Robot {
public:
    TwoLeggedRobot(raisim::World *world, std::string urdfPath, std::string name) : Robot(world, urdfPath, name) {
        initialize();
    }

    void initialize() override;
private:
    static const double deg2rad;
};


#endif //RAISIM_TWOLEGGEDROBOT_H
