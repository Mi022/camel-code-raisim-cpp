#include "SimplePendulumRobot.h"

//initialize initial position
void SimplePendulumRobot::initialize() {
    Eigen::VectorXd initialJointPosition(robot->getGeneralizedCoordinateDim());
    initialJointPosition.setZero();
    initialJointPosition[0] = 1.57;  //1.57rad
    robot->setGeneralizedCoordinate(initialJointPosition);
}