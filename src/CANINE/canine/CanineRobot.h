//
// Created by hs on 22. 8. 8.
//

#ifndef RAISIM_CANINEROBOT_H
#define RAISIM_CANINEROBOT_H

#include "include/CAMEL/Robot.h"

class CanineRobot : public Robot {
public:
    CanineRobot(raisim::World *world, std::string urdfPath, std::string name) : Robot(world, urdfPath, name) {
        initialize();
    }

    void initialize();
};



#endif //RAISIM_CANINEROBOT_H
