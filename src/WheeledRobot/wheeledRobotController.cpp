//
// Created by ljm on 22. 7. 6.
//

#include "wheeledRobotController.h"

void wheeledRobotController::doControl() {
    setControlInput();
}

void wheeledRobotController::setTrajectory() {

}

void wheeledRobotController::updateState() {
    position = getRobot()->robot->getGeneralizedCoordinate();
    velocity = getRobot()->robot->getGeneralizedVelocity();
}

void wheeledRobotController::computeControlInput() {

}

void wheeledRobotController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(torque);
    getRobot()->robot->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
}
