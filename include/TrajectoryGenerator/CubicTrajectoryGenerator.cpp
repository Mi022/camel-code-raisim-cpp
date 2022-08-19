//
// Created by jaehoon on 22. 6. 1.
//

#include "CubicTrajectoryGenerator.h"

#include <iostream>

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

void CubicTrajectoryGeneratorND::updateTrajectory(Eigen::VectorXd currentPosition, Eigen::VectorXd goalPosition, double currentTime, double timeDuration) {
    Eigen::VectorXd zeros = Eigen::VectorXd(mDim);
    zeros.setZero();

    mFunctionValue.rightCols(1) = currentPosition;
    mFunctionValue.conservativeResize(mFunctionValue.rows(), mFunctionValue.cols() + 1);
    mFunctionValue.rightCols(1) = goalPosition;
    mFunctionValue.conservativeResize(mFunctionValue.rows(), mFunctionValue.cols() + 1);
    mFunctionValue.rightCols(1) = zeros;
    mFunctionValue.conservativeResize(mFunctionValue.rows(), mFunctionValue.cols() + 1);
    mFunctionValue.rightCols(1) = zeros;
    mReferenceTime = currentTime;
    mTimeDuration = timeDuration;
    calculateCoefficient();
}

void CubicTrajectoryGeneratorND::calculateCoefficient() {
    mCoefficient = mFunctionValue * mMatrixA.transpose();
}

Eigen::VectorXd CubicTrajectoryGeneratorND::getPositionTrajectory(double currentTime) {
    if(currentTime>mReferenceTime + mTimeDuration) currentTime = mReferenceTime + mTimeDuration;
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    Eigen::Vector4d timeMatrix;
    timeMatrix << pow(normalizedTime, 3.0), pow(normalizedTime, 2.0), normalizedTime, 1.0;
    return mCoefficient*timeMatrix;
}

Eigen::VectorXd CubicTrajectoryGeneratorND::getVelocityTrajectory(double currentTime) {
    if(currentTime>mReferenceTime + mTimeDuration) currentTime = mReferenceTime + mTimeDuration;
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    Eigen::Vector4d timeMatrix;
    timeMatrix << 3*pow(normalizedTime, 2.0), 2*normalizedTime, 1.0, 0.0;
    return mCoefficient*timeMatrix;
}

Eigen::VectorXd CubicTrajectoryGeneratorND::getAccelerationTrajectory(double currentTime) {
    if(currentTime>mReferenceTime + mTimeDuration) currentTime = mReferenceTime + mTimeDuration;
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    Eigen::Vector4d timeMatrix;
    timeMatrix << 6*normalizedTime, 2.0, 0.0, 0.0;
    return mCoefficient*timeMatrix;
}

int CubicTrajectoryGeneratorND::getDim() const {
    return mDim;
}

