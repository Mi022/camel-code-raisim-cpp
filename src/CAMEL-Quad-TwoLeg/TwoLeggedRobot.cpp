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
    initialJointPosition[2] = 0.52;

    //rotation[quaternion]
    initialJointPosition[3] = 0.0;
    initialJointPosition[4] = 0.7071068;
    initialJointPosition[5] = 0.0;
    initialJointPosition[6] = 0.7071068;

    //BR-hip
    initialJointPosition[7] = 1.5707963;

    //BL-hip
    initialJointPosition[9] = 1.5707963;

    robot->setGeneralizedCoordinate(initialJointPosition);
}

raisim::VecDyn TwoLeggedRobot::getQ() {
    return this->robot->getGeneralizedCoordinate();//11 dim
}

raisim::VecDyn TwoLeggedRobot::getQD() {
    return this->robot->getGeneralizedVelocity();//10 dim
}