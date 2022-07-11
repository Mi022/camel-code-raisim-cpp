//
// Created by jaehoon on 22. 5. 2.
//

#include "SingleLeggedRobot.h"

void SingleLeggedRobot::initialize() {
    Eigen::VectorXd initialJointPosition(robot->getGeneralizedCoordinateDim());
    initialJointPosition.setZero();
//    initialJointPosition[0] = 0.23;  //prismatic joint
//    initialJointPosition[1] = 60.0 * 3.141592 / 180.0;
//    initialJointPosition[2] = -120.0 * 3.141592 / 180.0;
    robot->setGeneralizedCoordinate(initialJointPosition);
}

raisim::VecDyn SingleLeggedRobot::getQ() {
    return this->robot->getGeneralizedCoordinate();
}

raisim::VecDyn SingleLeggedRobot::getQD() {
    return this->robot->getGeneralizedVelocity();
}