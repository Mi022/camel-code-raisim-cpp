//
// Created by jaehoon on 22. 5. 2.
//

#include "DongdogsinglelegRobot.h"

void DongdogsinglelegRobot::initialize() {
    Eigen::VectorXd initialJointPosition(robot->getGeneralizedCoordinateDim());
    initialJointPosition.setZero();
    initialJointPosition[0] = 1.57;
    robot->setGeneralizedCoordinate(initialJointPosition);
}

double DongdogsinglelegRobot::getQ() {
    return this->robot->getGeneralizedCoordinate()[0];
}

double DongdogsinglelegRobot::getQD() {
    return this->robot->getGeneralizedVelocity()[0];
}