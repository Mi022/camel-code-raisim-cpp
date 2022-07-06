//
// Created by ljm on 22. 7. 6.
//

#include "wheeledRobot.h"

void wheeledRobot::initialize() {
    Eigen::VectorXd gc(robot->getGeneralizedCoordinateDim()), gv(robot->getDOF());
    gc.setZero(); gv.setZero();

    gc.segment<7>(0) << 0, 0, 0.197, 1, 0, 0, 0;

    robot->setGeneralizedCoordinate(gc);
    robot->setGeneralizedVelocity(gv);
}

raisim::VecDyn wheeledRobot::getQ() {
    return this->robot->getGeneralizedCoordinate();
}

raisim::VecDyn wheeledRobot::getQD() {
    return this->robot->getGeneralizedVelocity();
}
