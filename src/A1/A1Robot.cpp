//
// Created by jaehoon on 22. 5. 2.
//

#include "A1Robot.h"

void A1Robot::initialize() {
    Eigen::VectorXd initialJointPosition(robot->getGeneralizedCoordinateDim());
    std::cout << robot->getGeneralizedCoordinateDim() << std::endl;
    initialJointPosition.setZero();

    // base_x,y,z
    initialJointPosition[0] = 0.0;
    initialJointPosition[1] = 0.0;
    initialJointPosition[2] = 0.2964;

    // base_rotation [quaternion]
    initialJointPosition[3] = 1.0;
    initialJointPosition[4] = 0.0;
    initialJointPosition[5] = 0.0;
    initialJointPosition[6] = 0.0;

    // FR_hip,thigh,calf
    initialJointPosition[7] = 0.0;
    initialJointPosition[8] = 0.8683;
    initialJointPosition[9] = -1.5679;

    // FL_hip,thigh,calf
    initialJointPosition[10] = -0.0;
    initialJointPosition[11] = 0.8683;
    initialJointPosition[12] = -1.5679;

    // RR_hip,thigh,calf
    initialJointPosition[13] = 0.0;
    initialJointPosition[14] = 0.8683;
    initialJointPosition[15] = -1.5679;

    // RL_hip,thigh,calf
    initialJointPosition[16] = -0.0;
    initialJointPosition[17] = 0.8683;
    initialJointPosition[18] = -1.5679;

    robot->setGeneralizedCoordinate(initialJointPosition);
    robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
}

raisim::VecDyn A1Robot::getQ() {
    return this->robot->getGeneralizedCoordinate();
}

raisim::VecDyn A1Robot::getQD() {
    return this->robot->getGeneralizedVelocity();
}