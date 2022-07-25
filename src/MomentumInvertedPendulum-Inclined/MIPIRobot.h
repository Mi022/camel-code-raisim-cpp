//
// Created by user on 22. 6. 18.
//

#ifndef RAISIM_MIPIROBOT_H
#define RAISIM_MIPIROBOT_H

#include "include/CAMEL/Robot.h"

class MIPIRobot : public Robot {
public:
    MIPIRobot(raisim::World *world, std::string urdfPath, std::string name) : Robot(world, urdfPath, name) {
        initialize();
    }

    void initialize() override;

private:

};


#endif //RAISIM_MIPIROBOT_H
