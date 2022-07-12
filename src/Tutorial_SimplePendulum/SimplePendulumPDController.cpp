#include "SimplePendulumPDController.h"

// set PD gains of PD controller
void SimplePendulumPDController::setPDGain(double PGain, double DGain) {
    this->PGain = PGain;
    this->DGain = DGain;
}

// control algorithm
void SimplePendulumPDController::doControl() {
    setTrajectory();
    updateState();
    computeControlInput();
    setControlInput();
}

// set desired states of the robot
void SimplePendulumPDController::setTrajectory() {
    desiredPosition = mTrajectoryGenerator.getPositionTrajectory(getRobot()->getWorldTime());
    desiredVelocity = mTrajectoryGenerator.getVelocityTrajectory(getRobot()->getWorldTime());
}

// update states of the robot
void SimplePendulumPDController::updateState() {
    position = getRobot()->getQ();
    velocity = getRobot()->getQD();
}

// compute control inputs based on PD control method
void SimplePendulumPDController::computeControlInput() {
    positionError = desiredPosition - position[0];
    velocityError = desiredVelocity - velocity[0];
    torque[0] = PGain * positionError + DGain * velocityError;
}

// set computed force(torque) to the robot
void SimplePendulumPDController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(torque);
}