//
// Created by hwayoung on 22. 8. 29.
//

#include "IceCreamTestController.h"

void IceCreamTestController::doControl() {
    setTrajectory();
    updateState();
    computeControlInput();
//    setControlInput();
}

// set desired states of the robot
void IceCreamTestController::setTrajectory() {
}

// update states of the robot
void IceCreamTestController::updateState() {
    position = getRobot()->getQ().e();
    velocity = getRobot()->getQD().e();
}

// compute control inputs based on PD control method
void IceCreamTestController::computeControlInput() {
    torque.setZero();
}

// set computed force(torque) to the robot
void IceCreamTestController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(torque);
}