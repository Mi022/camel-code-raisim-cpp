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
    double d2r = 3.141592/180;

    desiredPosition[1] = 30*d2r;
    desiredPosition[2] = -60*d2r;
    desiredVelocity[1] = 0;
//        desiredPosition = mTrajectoryGenerator.getPositionTrajectory(getRobot()->getWorldTime());
//        desiredVelocity = mTrajectoryGenerator.getVelocityTrajectory(getRobot()->getWorldTime());


}

// update states of the robot
void JHdogPDController::updateState() {
    position = getRobot()->getQ();
    velocity = getRobot()->getQD();
}

// compute control inputs based on PD control method
void JHdogPDController::computeControlInput() {
    for(int i=1; i<3; i++)
    {
        positionError[i] = desiredPosition[i] - position[i];
        velocityError[i] = desiredVelocity[i] - velocity[i];
        torque[i] = PGain * positionError[i] + DGain * velocityError[i];
    }
}

// set computed force(torque) to the robot
void JHdogPDController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(torque);
}