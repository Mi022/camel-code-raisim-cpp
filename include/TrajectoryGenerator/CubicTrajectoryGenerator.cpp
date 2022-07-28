//
// Created by jaehoon on 22. 6. 1.
//

#include "CubicTrajectoryGenerator.h"
#include "iostream"

void CubicTrajectoryGenerator::updateTrajectory(double currentPosition, double goalPosition, double currentTime, double timeDuration) {
    mFunctionValue << currentPosition, goalPosition, 0.0, 0.0;
    mReferenceTime = currentTime;
    mTimeDuration = timeDuration;
    calculateCoefficient();
}

void CubicTrajectoryGenerator::calculateCoefficient() {
    mCoefficient = mMatrixA * mFunctionValue;
}

double CubicTrajectoryGenerator::getPositionTrajectory(double currentTime) {
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    std::cout << normalizedTime << std::endl;

    return mCoefficient(0,0) * pow(normalizedTime, 3.0) + mCoefficient(1,0) * pow(normalizedTime, 2.0) + mCoefficient(2,0) * normalizedTime + mCoefficient(3, 0);
}

double CubicTrajectoryGenerator::getVelocityTrajectory(double currentTime) {
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    return (3.0 *mCoefficient(0,0) * pow(normalizedTime, 2.0) + 2.0 * mCoefficient(1,0) * normalizedTime + mCoefficient(2,0)) / mTimeDuration;
}

double CubicTrajectoryGenerator::getAccelerationTrajectory(double currentTime) {
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    return (6.0 *mCoefficient(0,0) * normalizedTime + 2.0 * mCoefficient(1,0)) / pow(mTimeDuration, 2.0);
}