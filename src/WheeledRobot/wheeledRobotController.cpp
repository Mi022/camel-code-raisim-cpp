//
// Created by ljm on 22. 7. 6.
//

#include "wheeledRobotController.h"

void wheeledRobotController::doControl() {
    updateState();
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
    if(torque[6] < 30 && velocity[6] < vel) {
        torque[6] += 6;
    }
    else if(velocity[6] > vel) {
        torque[6] = 0;
    }

    if(torque[7] < 30 && velocity[7] < vel) {
        torque[7] += 6;
    }
    else if(velocity[7] > vel) {
        torque[7] = 0;
    }

    if(torque[8] < 30 && velocity[8] < vel) {
        torque[8] += 6;
    }
    else if(velocity[8] > vel) {
        torque[8] = 0;
    }

    if(torque[9] < 30 && velocity[9] < vel) {
        torque[9] += 6;
    }
    else if(velocity[9] > vel) {
        torque[9] = 0;
    }
}

void wheeledRobotController::setLeft() {
    updateState();

    if(torque[6] < 30 && velocity[6] < vel) {
        torque[6] += 6;
    }

    if(torque[7] > -30 && velocity[7] > vel) {
        torque[7] -= 3;
    }

    if(torque[8] < 30 && velocity[8] < vel) {
        torque[8] += 6;
    }

    if(torque[9] > -30 && velocity[9] > vel) {
        torque[9] -= 3;
    }
}

void wheeledRobotController::setRight() {
    updateState();

    if(torque[6] > -30 && velocity[6] > vel) {
        torque[6] -= 3;
    }

    if(torque[7] < 30 && velocity[7] < vel) {
        torque[7] += 6;
    }

    if(torque[8] > -30 && velocity[8] > vel) {
        torque[8] -= 3;
    }

    if(torque[9] < 30 && velocity[9] < vel) {
        torque[9] += 6;
    }
}

void wheeledRobotController::setBack() {
    updateState();

    if(torque[6] < 30 && velocity[6] > vel) {
        torque[6] -= 6;
    }
    else if(velocity[6] < vel) {
        torque[6] = 0;
    }

    if(torque[7] < 30 && velocity[7] > vel) {
        torque[7] -= 6;
    }
    else if(velocity[7] < vel) {
        torque[7] = 0;
    }

    if(torque[8] < 30 && velocity[8] > vel) {
        torque[8] -= 6;
    }
    else if(velocity[8] < vel) {
        torque[8] = 0;
    }

    if(torque[9] < 30 && velocity[9] > vel) {
        torque[9] -= 6;
    }
    else if(velocity[9] < vel) {
        torque[9] = 0;
    }
}

void wheeledRobotController::setStop() {
    computeControlInput();
}

void wheeledRobotController::accelerate() {
    vel = maxVel;
}

void wheeledRobotController::setVel(int val) {
    vel = 5 * val;
}

void wheeledRobotController::reset() {

}

void wheeledRobotController::setPGain(double PGain) {
    this->PGain = PGain;
}