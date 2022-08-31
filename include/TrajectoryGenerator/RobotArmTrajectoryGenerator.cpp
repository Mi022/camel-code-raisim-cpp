//
// Created by jy on 22. 6. 1.
//

#include "RobotArmTrajectoryGenerator.h"
#include "iostream"
#include "Eigen/Eigen"
#include "cmath"

void RobotArmTrajectoryGenerator::setWaypoints(Eigen::MatrixXd wayPoints) {
    mWaypoints = (3.141592 / 180) * wayPoints;
    pointNum = wayPoints.rows();
    calculateCoefficient();
}

void RobotArmTrajectoryGenerator::updateTrajectory(double currentTime, double timeDuration) {

    mFunctionValue.setZero();
    mReferenceTime = currentTime;
    mTimeDuration = timeDuration;
}

void RobotArmTrajectoryGenerator::calculateCoefficient() {
    std::cout << " way point " << std::endl;
    std::cout << mWaypoints << std::endl;
    mFunctionValue.row(0) = mWaypoints.row(0);
    mFunctionValue.row(1) = mWaypoints.row(1);

    for (int i = 0; i < 6; ++i) {
        mCoefficient.col(i) = mMatrixA * mFunctionValue.col(i);
    }

}

std::vector<double> RobotArmTrajectoryGenerator::getPositionTrajectory(double currentTime) {
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    std::vector<double> position(6);
    std::cout << " mCoefficient " << std::endl;
    std::cout << mCoefficient << std::endl;
    for (int i = 0; i < 6; ++i)
    {
        position[i] = mCoefficient(0, i) * pow(normalizedTime, 3.0) + mCoefficient(1, i) * pow(normalizedTime, 2.0) +
                      mCoefficient(2, i) * normalizedTime + mCoefficient(3, i);
    }
    return position;
}

std::vector<double> RobotArmTrajectoryGenerator::getVelocityTrajectory(double currentTime) {
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    std::vector<double> velocity(6);
    for (int i = 0; i < 6; ++i)
    {
        velocity[i] = (3.0 * mCoefficient(0, i) * pow(normalizedTime, 2.0) + 2.0 * mCoefficient(1, i) * normalizedTime +
                       mCoefficient(2, i)) / mTimeDuration;
    }
    return velocity;
}

//double RobotArmTrajectoryGenerator::getAccelerationTrajectory(double currentTime) {
//    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
//    return (6.0 *mCoefficient(0,0) * normalizedTime + 2.0 * mCoefficient(1,0)) / pow(mTimeDuration, 2.0);
//}
