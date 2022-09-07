//
// Created by jaehoon on 22. 5. 2.
//

#include "SingleLeggedRobot.h"

void SingleLeggedRobot::initialize() {
    Eigen::VectorXd initialJointPosition(robot->getGeneralizedCoordinateDim());
    initialJointPosition.setZero();
    initialJointPosition[0] = 0.23;  //prismatic joint
    initialJointPosition[1] = 60.0 * 3.141592 / 180.0;
    initialJointPosition[2] = -120.0 * 3.141592 / 180.0;
    robot->setGeneralizedCoordinate(initialJointPosition);
}

double SingleLeggedRobot::getGRFz() {
    double dT = 0.005;
    return robot -> getContacts()[0].getImpulse()[2]/dT;
}