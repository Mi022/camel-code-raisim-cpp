//
// Created by user on 22. 6. 9.
//

#ifndef RAISIM_MIPROBOT_H
#define RAISIM_MIPROBOT_H

#include "include/CAMEL/Robot.h"

//TODO:
class MIPRobot : public Robot {
public:
    MIPRobot(raisim::World *world, std::string urdfPath, std::string name) : Robot(world, urdfPath, name) {
        initialize();
    }

    void initialize() override;
    raisim::VecDyn getQ();
    raisim::VecDyn getQD();

private:

};


#endif //RAISIM_MIPROBOT_H
