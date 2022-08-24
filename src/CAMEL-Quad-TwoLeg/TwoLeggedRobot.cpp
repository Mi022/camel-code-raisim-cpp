
#include "TwoLeggedRobot.h"

void TwoLeggedRobot::initialize() {
    Eigen::VectorXd initialJointPosition(robot->getGeneralizedCoordinateDim());
    initialJointPosition.setZero();
    double l = 0.2;
    double theta1 = -30*deg2rad;
    double theta2 = 60*deg2rad;
//    std::cout<<theta2<<std::endl;
    //base position (x,y,z)
    initialJointPosition[0] = 0.0;  //prismatic joint
    initialJointPosition[1] = 0.0;
    initialJointPosition[2] = 0.1 + l*(cos(theta1+theta2) + cos(theta1));

    //rotation[quaternion]
    initialJointPosition[3] = 0.0;
    initialJointPosition[4] = 0.7071068;
    initialJointPosition[5] = 0.0;
    initialJointPosition[6] = 0.7071068;

    //BR-hip
    initialJointPosition[7] = 1.5707963 + theta1;
    //BR-knee
    initialJointPosition[8] = theta2;
    //BL-hip
    initialJointPosition[9] = 1.5707963 + theta1;
    //BL-knee
    initialJointPosition[10] = theta2;

    robot->setGeneralizedCoordinate(initialJointPosition);
}

const double TwoLeggedRobot::deg2rad = 3.141592/180;
