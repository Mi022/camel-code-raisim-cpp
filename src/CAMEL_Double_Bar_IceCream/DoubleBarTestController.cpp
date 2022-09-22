//
// Created by hwayoung on 22. 8. 29.
//

#include "DoubleBarTestController.h"

void DoubleBarTestController::doControl() {
    setTrajectory();
    updateState();
    computeControlInput();
    setControlInput();
}

// set desired states of the robot
void DoubleBarTestController::setTrajectory() {
    desiredPosition[1] = mTrajectoryGenerator.getPositionTrajectory(getRobot()->getWorldTime());
    desiredVelocity[1] = mTrajectoryGenerator.getVelocityTrajectory(getRobot()->getWorldTime());
}

// update states of the robot
void DoubleBarTestController::updateState() {
    PastVelocity = velocity;
    position = getRobot()->getQ().e();
    velocity = getRobot()->getQD().e();
    std::cout<<"QDDot : "<<std::endl<<(PastVelocity - velocity)/0.005<<std::endl;
}

// compute control inputs based on PD control method
void DoubleBarTestController::computeControlInput() {
    torque.setZero();
//    torque[1] = mPGain*(desiredPosition[1] - position[1]) + mDGain*(desiredVelocity[1] - velocity[1]);
    torque[0] = 30.0901;
    torque[1] = 22.9376;
    torque[2] = 10.2527;
}

// set computed force(torque) to the robot
void DoubleBarTestController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(torque);
    std::cout<<"Tau : "<<std::endl<<torque<<std::endl;
}