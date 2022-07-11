//
// Created by jaehoon on 22. 5. 2.
//

#include "SimplePendulumRobot.h"

void SimplePendulumRobot::initialize() {
    Eigen::VectorXd initialJointPosition(robot->getGeneralizedCoordinateDim());
    initialJointPosition.setZero();
    initialJointPosition[0] = 1.57;
    robot->setGeneralizedCoordinate(initialJointPosition);
}

double SimplePendulumRobot::getQ() {
    return this->robot->getGeneralizedCoordinate()[0];
}

double SimplePendulumRobot::getQD() {
    return this->robot->getGeneralizedVelocity()[0];
}