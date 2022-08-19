//
// Created by jaehyeong on 22. 8. 11.
//
#include "JHdog.h"


//initialize initial position
void JHdog::initialize(){
    Eigen::VectorXd initialJointPosition(robot->getGeneralizedCoordinateDim());
    initialJointPosition.setZero();
    initialJointPosition[0] = 0.5; // linear guide height
    initialJointPosition[1] = 0;
    initialJointPosition[2] = 0;
    robot->setGeneralizedCoordinate(initialJointPosition);
}
