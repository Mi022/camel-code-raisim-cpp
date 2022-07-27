//
// Created by jy on 22. 5. 2.
//

#include "RobotArmPDController.h"

void RobotArmPDController::setPDGain(Eigen::VectorXd PGain, Eigen::VectorXd DGain) {
    this->PGain = PGain;
    this->DGain = DGain;
}

void RobotArmPDController::doControl() {
    setTrajectory();
    updateState();
    computeControlInput();
    setControlInput();
}

void RobotArmPDController::setTrajectory() {
    positionTrajectory = mTrajectoryGenerator.getPositionTrajectory(getRobot()->getWorldTime());
    for(int i=0;i<6;i++){
        desiredPosition[i] = positionTrajectory[i];
    }
    desiredVelocity << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
}

void RobotArmPDController::updateState() {
    position = getRobot()->robot->getGeneralizedCoordinate();
    velocity = getRobot()->robot->getGeneralizedVelocity();
}

void RobotArmPDController::computeControlInput() {
    for (int i = 0; i < 6; i++) {
        positionError[i] = desiredPosition[i] - position[i];
        velocityError[i] = desiredVelocity[i] - velocity[i];
        torque[i] = (PGain[i] * positionError[i] + DGain[i] * velocityError[i]);
    };
}

void RobotArmPDController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(torque);

}
