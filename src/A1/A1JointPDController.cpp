//
// Created by jaehoon on 22. 5. 2.
//

#include "A1JointPDController.h"

void A1JointPDController::setPDGain(double PGain, double DGain) {
    this->PGain = PGain;
    this->DGain = DGain;
}

void A1JointPDController::doControl() {
    updateState();
    computeControlInput();
    setControlInput();
}

void A1JointPDController::setTrajectory() {
    // should be extended to 12 dof
//    this->desiredPosition = desiredPosition;
//    this->desiredVelocity = desiredVelocity;
}

void A1JointPDController::updateJointTrajectory(Eigen::VectorXd desiredPosition, Eigen::VectorXd desiredVelocity) {
    for (int i = 0; i < 12; i++) {
        this->desiredPosition[i] = desiredPosition[i];
        this->desiredVelocity[i] = desiredVelocity[i];
    }

}

void A1JointPDController::updateState() {
    position = getRobot()->robot->getGeneralizedCoordinate();
    velocity = getRobot()->robot->getGeneralizedVelocity();
}

void A1JointPDController::computeControlInput() {
    for (int i = 0; i < 12; i++) {
        positionError[i] = desiredPosition[i] - position[i + 7];
        velocityError[i] = desiredVelocity[i] - velocity[i + 6];
        torque[i + 6] = (PGain * positionError[i] + DGain * velocityError[i]);
        if (torque[i + 6] > torqueLimit) {
            torque[i + 6] = torqueLimit;
        } else if (torque[i + 6] < -torqueLimit) {
            torque[i + 6] = -torqueLimit;
        }
    }
}

void A1JointPDController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(torque);
}