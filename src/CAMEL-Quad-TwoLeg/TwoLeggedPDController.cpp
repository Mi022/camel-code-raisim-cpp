//
// Created by jaehoon on 22. 5. 2.
//

#include "TwoLeggedPDController.h"

<<<<<<< HEAD
void TwoLeggedPDController::setPDGain(double PGain, double DGain) {
=======
void SingleLeggedPDController::setPDGain(double PGain, double DGain) {
>>>>>>> initialize standup branch
    this->PGain = PGain;
    this->DGain = DGain;
}

<<<<<<< HEAD
void TwoLeggedPDController::doControl() {
=======
void SingleLeggedPDController::doControl() {
>>>>>>> initialize standup branch
    updateState();
    setTrajectory();
    IKsolve();
    computeControlInput();
    setControlInput();
}

<<<<<<< HEAD
void TwoLeggedPDController::setTrajectory() {
=======
void SingleLeggedPDController::setTrajectory() {
>>>>>>> initialize standup branch
    desiredPosition = mTrajectoryGenerator.getPositionTrajectory(getRobot()->getWorldTime());
    desiredVelocity = mTrajectoryGenerator.getVelocityTrajectory(getRobot()->getWorldTime());
}

<<<<<<< HEAD
void TwoLeggedPDController::updateState() {
=======
void SingleLeggedPDController::updateState() {
>>>>>>> initialize standup branch
    position = getRobot()->robot->getGeneralizedCoordinate();
    velocity = getRobot()->robot->getGeneralizedVelocity();
}

<<<<<<< HEAD
void TwoLeggedPDController::computeControlInput() {
=======
void SingleLeggedPDController::computeControlInput() {
>>>>>>> initialize standup branch
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

<<<<<<< HEAD
void TwoLeggedPDController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(torque);
}

void TwoLeggedPDController::IKsolve() {
=======
void SingleLeggedPDController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(torque);
}

void SingleLeggedPDController::IKsolve() {
>>>>>>> initialize standup branch
    desiredJointPosition[0] = acos(desiredPosition / 0.46);
    desiredJointPosition[1] = -2*desiredJointPosition[0];
    desiredJointVelocity.setZero();
}