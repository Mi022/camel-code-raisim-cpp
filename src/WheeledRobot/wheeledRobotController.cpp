//
// Created by ljm on 22. 7. 6.
//

#include "wheeledRobotController.h"

void wheeledRobotController::doControl() {
    updateState();
    computeControlInput();
    setControlInput();
}

void wheeledRobotController::setTrajectory() {

}

void wheeledRobotController::updateState() {
    position = getRobot()->robot->getGeneralizedCoordinate();
    velocity = getRobot()->robot->getGeneralizedVelocity();
}

void wheeledRobotController::computeControlInput() {
    for (int i = 0; i < 4; i++) {
        velocityError[i] = desiredVelocity[i] - velocity[i+6];
        torque[i+6] = (PGain * velocityError[i]);
        if (torque[i+6] > torqueLimit) {
            torque[i+6] = torqueLimit;
        } else if (torque[i+6] < -torqueLimit) {
            torque[i+6] = -torqueLimit;
        }
    }
}

void wheeledRobotController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(torque);
    getRobot()->robot->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
}

void wheeledRobotController::setForward() {
    if(vel) {
        desiredVelocity[0] = vel;
        desiredVelocity[1] = vel;
        desiredVelocity[2] = vel;
        desiredVelocity[3] = vel;
    }
    else {
        desiredVelocity[0] = 15;
        desiredVelocity[1] = 15;
        desiredVelocity[2] = 15;
        desiredVelocity[3] = 15;
    }
}

void wheeledRobotController::setLeft() {
    if(vel) {
        desiredVelocity[0] = 2 * vel;
        desiredVelocity[1] = vel;
        desiredVelocity[2] = 2 * vel;
        desiredVelocity[3] = vel;
    }
    else {
        desiredVelocity[0] = 30;
        desiredVelocity[1] = 15;
        desiredVelocity[2] = 30;
        desiredVelocity[3] = 15;
    }
}

void wheeledRobotController::setRight() {
    if(vel) {
        desiredVelocity[0] = vel;
        desiredVelocity[1] = 2 * vel;
        desiredVelocity[2] = vel;
        desiredVelocity[3] = 2 * vel;
    }
    else {
        desiredVelocity[0] = 15;
        desiredVelocity[1] = 30;
        desiredVelocity[2] = 15;
        desiredVelocity[3] = 30;
    }
}

void wheeledRobotController::setBack() {
    if(vel) {
        desiredVelocity[0] = vel;
        desiredVelocity[1] = vel;
        desiredVelocity[2] = vel;
        desiredVelocity[3] = vel;
    }
    else {
        desiredVelocity[0] = -15;
        desiredVelocity[1] = -15;
        desiredVelocity[2] = -15;
        desiredVelocity[3] = -15;
    }
}

void wheeledRobotController::setStop() {
    desiredVelocity.setZero();
}

void wheeledRobotController::accelerate() {
    vel = velLimit;
}

void wheeledRobotController::setVel(int val) {
    vel = -5 * val;
}

void wheeledRobotController::reset() {

}

void wheeledRobotController::setPGain(double PGain) {
    this->PGain = PGain;
}