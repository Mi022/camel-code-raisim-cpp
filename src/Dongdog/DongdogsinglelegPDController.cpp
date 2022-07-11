//
// Created by jaehoon on 22. 5. 2.
//

#include "DongdogsinglelegPDController.h"

void DongdogsinglelegPDController::setPDGain(double PGain, double DGain) {
    this->PGain = PGain;
    this->DGain = DGain;
}

void DongdogsinglelegPDController::doControl() {
    setTrajectory();
    updateState();
    computeControlInput();
    setControlInput();
}

void DongdogsinglelegPDController::setTrajectory() {
    desiredPosition = mTrajectoryGenerator.getPositionTrajectory(getRobot()->getWorldTime());
    desiredVelocity = mTrajectoryGenerator.getVelocityTrajectory(getRobot()->getWorldTime());
}

void DongdogsinglelegPDController::updateState() {
    position = getRobot()->robot->getGeneralizedCoordinate()[0];
    velocity = getRobot()->robot->getGeneralizedVelocity()[0];
}

void DongdogsinglelegPDController::computeControlInput() {
    positionError = desiredPosition - position;
    velocityError = desiredVelocity - velocity;
    torque[0] = PGain * positionError + DGain * velocityError;
}

void DongdogsinglelegPDController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(torque);
}
