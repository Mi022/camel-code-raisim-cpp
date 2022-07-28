//
// Created by jy on 22. 6. 1.
//

#include "RobotArmTrajectoryGenerator.h"
#include "iostream"

void RobotArmTrajectoryGenerator::setWaypoints(Eigen::MatrixXd wayPoints) {
    mWaypoints = wayPoints;
//    std::cout << mWaypoints << std::endl;

}

void RobotArmTrajectoryGenerator::updateTrajectory(double currentPosition, double goalPosition, double currentTime, double timeDuration) {
    mFunctionValue << currentPosition, goalPosition, 0.0, 0.0;
    mReferenceTime = currentTime;
    mTimeDuration = timeDuration;
    calculateCoefficient();
}

void RobotArmTrajectoryGenerator::calculateCoefficient() {
    mCoefficient = mMatrixA * mFunctionValue;
//    std::cout << mWaypoints << std::endl;

}

std::vector<double> RobotArmTrajectoryGenerator::getPositionTrajectory(double currentTime) {
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    std::vector<double> positionTrajectory(6);
//    std::cout << mWaypoints << std::endl;

    positionTrajectory[0] = 0;

    positionTrajectory[1] = 2;

    positionTrajectory[2] = mCoefficient(0,0) * pow(normalizedTime, 3.0) + mCoefficient(1,0) * pow(normalizedTime, 2.0)
                            + mCoefficient(2,0) * normalizedTime + mCoefficient(3, 0);

    positionTrajectory[3] = mCoefficient(0,0) * pow(normalizedTime, 3.0) + mCoefficient(1,0) * pow(normalizedTime, 2.0)
                            + mCoefficient(2,0) * normalizedTime + mCoefficient(3, 0);

    positionTrajectory[4] = mCoefficient(0,0) * pow(normalizedTime, 3.0) + mCoefficient(1,0) * pow(normalizedTime, 2.0)
                            + mCoefficient(2,0) * normalizedTime + mCoefficient(3, 0);

    positionTrajectory[5] = mCoefficient(0,0) * pow(normalizedTime, 3.0) + mCoefficient(1,0) * pow(normalizedTime, 2.0)
                            + mCoefficient(2,0) * normalizedTime + mCoefficient(3, 0);

    return positionTrajectory;
}

double RobotArmTrajectoryGenerator::getVelocityTrajectory(double currentTime) {
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    return (3.0 *mCoefficient(0,0) * pow(normalizedTime, 2.0) + 2.0 * mCoefficient(1,0) * normalizedTime + mCoefficient(2,0)) / mTimeDuration;
}
//
//double RobotArmTrajectoryGenerator::getAccelerationTrajectory(double currentTime) {
//    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
//    return (6.0 *mCoefficient(0,0) * normalizedTime + 2.0 * mCoefficient(1,0)) / pow(mTimeDuration, 2.0);
//}
