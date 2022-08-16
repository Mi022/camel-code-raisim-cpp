//
// Created by jaehyeong on 22. 8. 16.
//
#include "JHdogPDController.h"

void JHdogPDController:: setPDGain(double PGain, double DGain) {
    this->PGain = PGain;
    this->DGain = DGain;
}

void JHdogPDController:: doControl() {
    setTrajectory();
    updateState();
    computeControlInput();
    setControlInput();
}

// set desired states of the robot
void JHdogPDController::setTrajectory() {
    desiredPosition = mTrajectoryGenerator.getPositionTrajectory(getRobot()->getWorldTime());
    desiredVelocity = mTrajectoryGenerator.getVelocityTrajectory(getRobot()->getWorldTime());
}

// update states of the robot
void JHdogPDController::updateState() {
    position = getRobot()->getQ();
    velocity = getRobot()->getQD();
}

// compute control inputs based on PD control method
void JHdogPDController::computeControlInput() {
    positionError = desiredPosition - position[0];
    velocityError = desiredVelocity - velocity[0];
    torque[0] = PGain * positionError + DGain * velocityError;
}

// set computed force(torque) to the robot
void JHdogPDController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(torque);
}