//
// Created by user on 22. 6. 14.
//

#include "MIPPDDController.h"

void MIPPDDController::setPDDGain(double PGain, double DGain, double DDGain) {
    this->mPGain = PGain;
    this->mDGain = DGain;
    this->mDDGain = DDGain;
}

void MIPPDDController::generateExternalForce() {
    raisim::Vec<3> externalForce = {0.0, 9.6, 0.0};
    raisim::Vec<3> forcePosition = {0.0, 0.0, 0.08};

    if(mIteration%FORCE_DURATION == 0 || mIteration == 0){
        getRobot()->robot->setExternalForce(1, forcePosition, externalForce);
    }
    mIteration++;
}

void MIPPDDController::addNoise() {
    std::random_device rd;

    // random_device 를 통해 난수 생성 엔진을 초기화 한다.
    std::mt19937 gen(rd());

    // -100 부터 100 까지 균등하게 나타나는 난수열을 생성하기 위해 균등 분포 정의.
    std::uniform_int_distribution<int> dis(-RANDOM_BOUNDARY, RANDOM_BOUNDARY);
    double noisePosition = (double(dis(gen)) / RANDOM_BOUNDARY ) * MAX_POSITION_NOISE;
    double noiseVelocity = (double(dis(gen)) / RANDOM_BOUNDARY ) * MAX_VELOCITY_NOISE;

    mPosition[0] += noisePosition;
    mVelocity[0] += noiseVelocity;

}

void MIPPDDController::doControl() {
    updateState();
    addNoise();
    generateExternalForce();
    computeControlInput();
    setControlInput();
}

void MIPPDDController::setTrajectory() {
    mDesiredJointPosition[0] = 0.0;
    mDesiredJointVelocity[0] = 0.0;
}

void MIPPDDController::updateState() {
    mPosition = getRobot()->robot->getGeneralizedCoordinate();
    mVelocity = getRobot()->robot->getGeneralizedVelocity();
}

void MIPPDDController::computeControlInput() {
    mPositionError[0] = mDesiredJointPosition[0] - mPosition[0];
    mVelocityError[0] = mDesiredJointVelocity[0] - mVelocity[0];
    mMotorVelocityError = mDesiredMotorVelocity - mVelocity[1];
    mTorque[1] = -mPGain * mPositionError[0] - mDGain * mVelocityError[0] - mDDGain * mMotorVelocityError;

    if(mTorque[1] > mTorqueLimit)
    {
        mTorque[1] = mTorqueLimit;
    }
    else if(mTorque[1] < -mTorqueLimit)
    {
        mTorque[1] = -mTorqueLimit;
    }
}

void MIPPDDController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(mTorque);
}

const Eigen::VectorXd &MIPPDDController::getTorque() const {
    return mTorque;
}

const Eigen::VectorXd &MIPPDDController::getDesiredJointPosition() const {
    return mDesiredJointPosition;
}

const Eigen::VectorXd &MIPPDDController::getDesiredJointVelocity() const {
    return mDesiredJointVelocity;
}

double MIPPDDController::getDesiredMotorVelocity() const {
    return mDesiredMotorVelocity;
}
