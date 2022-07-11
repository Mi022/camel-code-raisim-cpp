//
// Created by jaehoon on 22. 5. 2.
//

#include "TwoLeggedRobot.h"

void TwoLeggedRobot::initialize() {
    Eigen::VectorXd initialJointPosition(robot->getGeneralizedCoordinateDim());
    initialJointPosition.setZero();
    //base position (x,y,z)
    initialJointPosition[0] = 0.0;  //prismatic joint
    initialJointPosition[1] = 0.0;
    initialJointPosition[2] = 1.0;

    //rotation[quaternion]
    initialJointPosition[3] = 1.0;
    initialJointPosition[4] = 0.0;
    initialJointPosition[5] = 0.0;
    initialJointPosition[6] = 0.0;
    robot->setGeneralizedCoordinate(initialJointPosition);
}

raisim::VecDyn TwoLeggedRobot::getQ() {
    return this->robot->getGeneralizedCoordinate();
}

raisim::VecDyn TwoLeggedRobot::getQD() {
    return this->robot->getGeneralizedVelocity();
}