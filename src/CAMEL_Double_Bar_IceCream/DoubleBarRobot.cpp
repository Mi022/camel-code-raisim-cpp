//
// Created by hwayoung on 22. 8. 29.
//

#include "DoubleBarRobot.h"

void DoubleBarRobot::initialize() {
    Eigen::VectorXd initialJointPosition(robot->getGeneralizedCoordinateDim());
    Eigen::VectorXd initialJointVelocity(robot->getGeneralizedCoordinateDim());
    std::cout<<initialJointPosition.size()<<std::endl;
    initialJointPosition.setZero();
    initialJointVelocity.setZero();
    double kneeAngle = 30;

    //base-pitch
    initialJointPosition[0] = kneeAngle*deg2rad;
    //knee-pitch
    initialJointPosition[1] = -2*kneeAngle*deg2rad;
    //hip-pitch
    initialJointPosition[2] = -(90-kneeAngle)*deg2rad;

    robot->setGeneralizedCoordinate(initialJointPosition);
    robot->setGeneralizedVelocity(initialJointVelocity);
}

const double DoubleBarRobot::deg2rad = 3.141592 / 180;