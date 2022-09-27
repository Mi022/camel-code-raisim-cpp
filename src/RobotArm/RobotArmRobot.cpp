//
// Created by jy on 22. 5. 2.
//

#include "RobotArmRobot.h"

void RobotArmRobot::initialize() {
    Eigen::VectorXd initialJointPosition(robot->getGeneralizedCoordinateDim());
    initialJointPosition << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    robot->setGeneralizedCoordinate(initialJointPosition);
}

raisim::VecDyn RobotArmRobot::getQ() {
    return this->robot->getGeneralizedCoordinate();
}

raisim::VecDyn RobotArmRobot::getQD() {
    return this->robot->getGeneralizedVelocity();
}