//
// Created by jaehyeong on 22. 8. 11.
//
#include "JHdog.h"


//initialize initial position
void JHdog::initialize(){
    Eigen::VectorXd initialJointPosition(robot->getGeneralizedCoordinateDim());
    initialJointPosition.setZero();
    initialJointPosition[0] = 1.57;  //1.57rad
    robot->setGeneralizedCoordinate(initialJointPosition);
}
