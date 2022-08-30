//
// Created by hwayoung on 22. 8. 29.
//

#include "IceCreamRobot.h"

void IceCreamRobot::initialize() {
    Eigen::VectorXd initialJointPosition(robot->getGeneralizedCoordinateDim());
    std::cout<<initialJointPosition.size()<<std::endl;
    initialJointPosition.setZero();
    double l = 0.2;
    double theta1 = 30*deg2rad;
    double theta2 = -60*deg2rad;

    //base-pitch
//    initialJointPosition[0] = 30*deg2rad;
    //hip-pitch
//    initialJointPosition[1] = -120*deg2rad;

//    //base-pitch
//    initialJointPosition[0] = 30*deg2rad;
//    //knee-pitch
//    initialJointPosition[1] = -60*deg2rad;
//    //hip-pitch
//    initialJointPosition[2] = 30*deg2rad;


    robot->setGeneralizedCoordinate(initialJointPosition);
}

//const double IceCreamRobot::deg2rad = 3.141592/180;