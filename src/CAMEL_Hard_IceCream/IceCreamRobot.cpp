//
// Created by hwayoung on 22. 8. 29.
//

#include "IceCreamRobot.h"

void IceCreamRobot::initialize() {
    Eigen::VectorXd initialJointPosition(robot->getGeneralizedCoordinateDim());
    Eigen::VectorXd initialJointVelocity(robot->getGeneralizedCoordinateDim());
    std::cout<<initialJointPosition.size()<<std::endl;
    initialJointPosition.setZero();
    initialJointVelocity.setZero();
    double l = 0.2;
    double theta1 = 30*deg2rad;
    double theta2 = -60*deg2rad;

    initialJointPosition = diffWay();

    robot->setGeneralizedCoordinate(initialJointPosition);
    robot->setGeneralizedVelocity(initialJointVelocity);
}

Eigen::VectorXd IceCreamRobot::diffWay() {
    Eigen::VectorXd initialJointPosition(robot->getGeneralizedCoordinateDim());
    //base-pitch
    initialJointPosition[0] = 10*deg2rad;
    //hip-pitch
    initialJointPosition[1] = -120*deg2rad;
    return initialJointPosition;
}

Eigen::VectorXd IceCreamRobot::sameWay() {
    Eigen::VectorXd initialJointPosition(robot->getGeneralizedCoordinateDim());
    double tiltAngle = 2.2;
    //base-pitch
    initialJointPosition[0] = tiltAngle*deg2rad;
    //hip-pitch
    initialJointPosition[1] = -(90+tiltAngle)*deg2rad;
    return initialJointPosition;
}
const double IceCreamRobot::deg2rad = 3.141592/180;