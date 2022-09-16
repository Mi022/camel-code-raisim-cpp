//
// Created by jaehoon on 22. 5. 2.
//

#include "SingleLeggedRobot.h"

/*void SingleLeggedRobot::initialize() {
    Eigen::VectorXd initialJointPosition(robot->getGeneralizedCoordinateDim());
    initialJointPosition.setZero();
    initialJointPosition[0] = 0.23;  //prismatic joint
    initialJointPosition[1] = 60.0 * 3.141592 / 180.0;
    initialJointPosition[2] = -120.0 * 3.141592 / 180.0;
    robot->setGeneralizedCoordinate(initialJointPosition);
}*/

void SingleLeggedRobot::initialize() {
    Eigen::VectorXd initialJointPosition(robot->getGeneralizedCoordinateDim());
    initialJointPosition.setZero();
    initialJointPosition[0] = 0.7;
    initialJointPosition[1] = -1.4;
    robot->setGeneralizedCoordinate(initialJointPosition);
}