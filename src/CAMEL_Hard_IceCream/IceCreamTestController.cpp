//
// Created by hwayoung on 22. 8. 29.
//

#include "IceCreamTestController.h"

void IceCreamTestController::doControl() {
    setTrajectory();
    updateState();
    computeControlInput();
    setControlInput();
}

// set desired states of the robot
void IceCreamTestController::setTrajectory() {
    desiredPosition[1] = mTrajectoryGenerator.getPositionTrajectory(getRobot()->getWorldTime());
    desiredVelocity[1] = mTrajectoryGenerator.getVelocityTrajectory(getRobot()->getWorldTime());
}

// update states of the robot
void IceCreamTestController::updateState() {
    position = getRobot()->getQ().e();
    velocity = getRobot()->getQD().e();
}

// compute control inputs based on PD control method
void IceCreamTestController::computeControlInput() {
    torque.setZero();
    torque[1] = mPGain*(desiredPosition[1] - position[1]) + mDGain*(desiredVelocity[1] - velocity[1]);
//    torque[1] = -0.1;
}

// set computed force(torque) to the robot
void IceCreamTestController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(torque);
}