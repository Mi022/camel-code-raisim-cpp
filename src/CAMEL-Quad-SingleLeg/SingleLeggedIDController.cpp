//
// Created by jaehoon on 22. 6. 6.
//

#include "SingleLeggedIDController.h"

void SingleLeggedIDController::setPDGain(double PGain, double DGain) {
    this->PGain = PGain;
    this->DGain = DGain;
}

void SingleLeggedIDController::doControl() {
    updateState();
    setTrajectory();
//    IKsolve();
    computeControlInput();
    setControlInput();
}

void SingleLeggedIDController::setTrajectory() {
    desiredPosition = mTrajectoryGenerator.getPositionTrajectory(getRobot()->getWorldTime());
    desiredVelocity = mTrajectoryGenerator.getVelocityTrajectory(getRobot()->getWorldTime());
    desiredAcceleration = mTrajectoryGenerator.getAccelerationTrajectory(getRobot()->getWorldTime());
}

void SingleLeggedIDController::updateState() {
    position = getRobot()->getQ();
    velocity = getRobot()->getQD();
}

void SingleLeggedIDController::computeControlInput() {
    calculatedForce = mLumpedMass * (desiredAcceleration + PGain * (desiredPosition - position[0]) + DGain * (desiredVelocity - velocity[0]) - mGravity);
    dz_dth1 = -0.23*sin(position[1]) - 0.23*sin(position[1] + position[2]);
    dz_dth2 = -0.23*sin(position[1] + position[2]);
    torque[1] = dz_dth1 * calculatedForce;
    torque[2] = dz_dth2 * calculatedForce;
}

void SingleLeggedIDController::setControlInput() {
    for (int i = 0; i < 3; i++) {
        if(torque[i] > torqueLimit)
        {
            torque[i] = torqueLimit;
        }
        else if(torque[i] < -torqueLimit)
        {
            torque[i] = -torqueLimit;
        }
    }
    getRobot()->robot->setGeneralizedForce(torque);
}

void SingleLeggedIDController::IKsolve() {
    desiredJointPosition_past = desiredJointPosition;
    desiredJointPosition[0] = acos(desiredPosition / 0.46);
    desiredJointPosition[1] = -2*desiredJointPosition[0];
    desiredJointVelocity = (desiredJointPosition - desiredJointPosition_past) / mDT;
}