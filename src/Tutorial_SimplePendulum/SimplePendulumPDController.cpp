//
// Created by jaehoon on 22. 5. 2.
//

#include "SimplePendulumPDController.h"

void SimplePendulumPDController::setPDGain(double PGain, double DGain) {
    this->PGain = PGain;
    this->DGain = DGain;
}

void SimplePendulumPDController::doControl() {
    setTrajectory();
    updateState();
    computeControlInput();
    setControlInput();
}

void SimplePendulumPDController::setTrajectory() {
    desiredPosition = mTrajectoryGenerator.getPositionTrajectory(getRobot()->getWorldTime());
    desiredVelocity = mTrajectoryGenerator.getVelocityTrajectory(getRobot()->getWorldTime());
}

void SimplePendulumPDController::updateState() {
    position = getRobot()->getQ();
    velocity = getRobot()->getQD();
}

void SimplePendulumPDController::computeControlInput() {
    positionError = desiredPosition - position[0];
    velocityError = desiredVelocity - velocity[0];
    torque[0] = PGain * positionError + DGain * velocityError;
}

void SimplePendulumPDController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(torque);
}
