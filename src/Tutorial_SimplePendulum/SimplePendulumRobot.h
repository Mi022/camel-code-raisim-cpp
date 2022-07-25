#include "include/CAMEL/Robot.h"     //include Robot class

// extend Robot class (parent) to SimplePendulumRobot class (child)
class SimplePendulumRobot : public Robot {
public:
    //initialize
    SimplePendulumRobot(raisim::World *world, std::string urdfPath, std::string name) : Robot(world, urdfPath, name) {
        initialize();
    }

    void initialize() override;      //override abstract method in robot
};