//
// Created by jaehoon on 22. 5. 2.
//

#include "SingleLeggedPDController.h"

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
    position = getRobot()->getQ();
    velocity = getRobot()->getQD();
    velocity = getNoisyQD();
}

raisim::VecDyn SingleLeggedPDController::getNoisyQD() {
    mNoisyQD[0] = velocity[0] + mDistribution(mGenerator);
    mNoisyQD[1] = velocity[1] + mDistribution(mGenerator);
    mNoisyQD[2] = velocity[2] + mDistribution(mGenerator);
    mFilteredQD[0] = mLPF1.doFiltering(mNoisyQD[0]);
    mFilteredQD[1] = mLPF2.doFiltering(mNoisyQD[1]);
    mFilteredQD[2] = mLPF2.doFiltering(mNoisyQD[2]);
    return mFilteredQD;
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
    desiredJointVelocity[0] = -desiredVelocity / (0.46 * sin(desiredJointPosition[0]));
    desiredJointVelocity[1] = -2*desiredJointVelocity[0];
}