//
// Created by hwayoung on 22. 8. 17.
//

#include "SingleLegRobot.h"
void SingleLegRobot::initialize() {
    Eigen::VectorXd initialJointPosition(robot->getGeneralizedCoordinateDim());
    std::cout<<initialJointPosition.size()<<std::endl;
    initialJointPosition.setZero();
    double l = 0.2;
    double theta1 = 30*deg2rad;
    double theta2 = -60*deg2rad;

    //base position (x,y,z)
    initialJointPosition[0] = 0.0;  //prismatic joint
    initialJointPosition[1] = 0.0;
    initialJointPosition[2] = l*(cos(theta1+theta2) + cos(theta1)) + 0.005; //0.4

    //rotation[quaternion]
    initialJointPosition[3] = 0.0;
    initialJointPosition[4] = 0.0;
    initialJointPosition[5] = 0.0;
    initialJointPosition[6] = 1.0;

    //BR-hip
    initialJointPosition[7] = theta1;
    //BR-knee
    initialJointPosition[8] = theta2;

    robot->setGeneralizedCoordinate(initialJointPosition);
}

const double SingleLegRobot::deg2rad = 3.141592/180;
