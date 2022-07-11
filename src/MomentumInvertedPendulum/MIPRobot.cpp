//
// Created by user on 22. 6. 9.
//

#include "MIPRobot.h"
// [0] : base-roll
// [1] : motor-revolute
void MIPRobot::initialize() {
    Eigen::VectorXd initialJointPosition(robot->getGeneralizedCoordinateDim());
    initialJointPosition.setZero();
    initialJointPosition[0] = 0.1; //0.4 for configuration figure
    robot->setGeneralizedCoordinate(initialJointPosition);
}

raisim::VecDyn MIPRobot::getQ() {
    return this->robot->getGeneralizedCoordinate();
}

raisim::VecDyn MIPRobot::getQD() {
    return this->robot->getGeneralizedVelocity();
}