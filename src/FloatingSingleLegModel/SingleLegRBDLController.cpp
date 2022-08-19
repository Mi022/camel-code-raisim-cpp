//
// Created by hwayoung on 22. 8. 17.
//

#include "SingleLegRBDLController.h"

void SingleLegRBDLController::doControl() {
    setTrajectory();
    updateState();
    computeControlInput();
    setControlInput();
}

// set desired states of the robot
void SingleLegRBDLController::setTrajectory() {
    if((position != desiredPosition || velocity != desiredVelocity) && !mIsTrajectoryAlready){
        position << 3.0, 3.0, 3.0, 0.0, 0.0, 0.0, 1.0, 3.0, 3.0;
        std::cout<<"currentPostion" << std::endl << position<< std::endl;
        std::cout<<"goalPostion" << std::endl << desiredPosition<< std::endl;
        mIsGenerateTrajectory = true;
    }
    if(mIsGenerateTrajectory){
        Eigen::VectorXd currentPosition, goalPosition;
        currentPosition = Eigen::VectorXd (mCubicTrajectoryGeneratorNd.getDim());
        goalPosition = Eigen::VectorXd (mCubicTrajectoryGeneratorNd.getDim());
        currentPosition << position[0], position[1], position[2], position[7], position[8];
        goalPosition << desiredPosition[0], desiredPosition[1], desiredPosition[2], desiredPosition[7], desiredPosition[8];
        mCubicTrajectoryGeneratorNd.updateTrajectory(currentPosition, goalPosition, getRobot()->getWorldTime(), mTimeDuration);
        mIsGenerateTrajectory = false;
        mIsTrajectoryAlready = true;
    }
    if(!mIsGenerateTrajectory && mIsTrajectoryAlready){
        desiredPosition[7] = mCubicTrajectoryGeneratorNd.getPositionTrajectory(getRobot()->getWorldTime())[3];
        std::cout<< "hi" << std::endl;
        std::cout<< "desiredPosition[7]: " << desiredPosition[7] <<std::endl;
    }
}

// update states of the robot
void SingleLegRBDLController::updateState() {
    position = getRobot()->getQ().e();
    velocity = getRobot()->getQD().e();
}

// compute control inputs based on PD control method
void SingleLegRBDLController::computeControlInput() {
    torque.setZero();
}

// set computed force(torque) to the robot
void SingleLegRBDLController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(torque);
}

