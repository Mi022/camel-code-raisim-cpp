//
// Created by jaehoon on 22. 5. 2.
//

#include "TwoLeggedPDController.h"

void SingleLeggedPDController::setPDGain(double PGain, double DGain) {
    this->PGain = PGain;
    this->DGain = DGain;
}

void SingleLeggedPDController::doControl() {
    updateState();
    setTrajectory();
    IKsolve();
    computeControlInput();
    setControlInput();
}

void SingleLeggedPDController::setTrajectory() {
    desiredPosition = mTrajectoryGenerator.getPositionTrajectory(getRobot()->getWorldTime());
    desiredVelocity = mTrajectoryGenerator.getVelocityTrajectory(getRobot()->getWorldTime());
}

void SingleLeggedPDController::updateState() {
    position = getRobot()->robot->getGeneralizedCoordinate();
    velocity = getRobot()->robot->getGeneralizedVelocity();
}

void SingleLeggedPDController::computeControlInput() {
    for (int i = 1; i < 3; i++) {
        positionError[i - 1] = desiredJointPosition[i - 1] - position[i];
        velocityError[i - 1] = desiredJointVelocity[i - 1] - velocity[i];
        torque[i] = PGain * positionError[i - 1] + DGain * velocityError[i - 1];
        if(torque[i] > torqueLimit)
        {
            torque[i] = torqueLimit;
        }
        else if(torque[i] < -torqueLimit)
        {
            torque[i] = -torqueLimit;
        }
    }
}

void SingleLeggedPDController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(torque);
}

void SingleLeggedPDController::IKsolve() {
    desiredJointPosition[0] = acos(desiredPosition / 0.46);
    desiredJointPosition[1] = -2*desiredJointPosition[0];
    desiredJointVelocity.setZero();
}