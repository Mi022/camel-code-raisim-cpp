//
// Created by hs on 22. 9. 16.
//

#include "SwingLegController.h"

void SwingLegController::doControl() {
    updateState();
    setTrajectory();
    IKsolve();
    computeControlInput();
    setControlInput();
    auto FRfootFrameIndex = getRobot()->robot->getFrameIdxByName("RF_FOOT");
    getRobot()->robot->getFramePosition(FRfootFrameIndex, footPosition);
}

void SwingLegController::setTrajectory() {
    mTrajectoryGenerator.getPositionTrajectory(getRobot()->getWorldTime());
    desiredPosition[1] = mTrajectoryGenerator.sumZ;
    desiredPosition[0] = mTrajectoryGenerator.sumX;
    //desiredVelocity = mTrajectoryGenerator.getVelocityTrajectory(getRobot()->getWorldTime());
}

void SwingLegController::updateState() {
    position = getRobot()->getQ();
    velocity = getRobot()->getQD();
}

void SwingLegController::IKsolve() {
    double d = sqrt(pow(desiredPosition[0],2)+pow(desiredPosition[1],2));
    double phi = acos(abs(desiredPosition[0])/ d);
    double psi = acos(pow(d,2)/(2*0.23*d));

    if (desiredPosition[0] < 0)
        desiredJointPosition[0] = 1.57 - phi + psi;
    else if(desiredPosition[0] = 0)
        desiredJointPosition[0] = psi;
    else
        desiredJointPosition[0] = phi + psi - 1.57;
    desiredJointPosition[1] = -acos((pow(d,2)-2*pow(0.23,2)) / (2*0.23*0.23));

    desiredJointVelocity.setZero();
}

void SwingLegController::computeControlInput() {
    for (int i = 0; i < 2; i++) {
        positionError[i] = desiredJointPosition[i] - position[i];
        velocityError[i] = desiredJointVelocity[i] - velocity[i];
        torque[i] = PGain[i] * positionError[i] + DGain[i] * velocityError[i];
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

void SwingLegController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(torque);
}

