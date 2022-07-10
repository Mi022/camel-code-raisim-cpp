//
// Created by user on 22. 6. 10.
//

#include "MIPPDController.h"

void MIPPDController::setPDGain(double PGain, double DGain) {
    this->mPGain = PGain;
    this->mDGain = DGain;
}

void MIPPDController::generateExternalForce() {
    raisim::Vec<3> externalForce = {0.0, 1.9, 0.0};
    raisim::Vec<3> forcePosition = {0.0, 0.0, 0.08};

    if(mIteration%FORCE_DURATION == 0 || mIteration == 0){
        getRobot()->robot->setExternalForce(1, forcePosition, externalForce);
    }
    mIteration++;
}

void MIPPDController::addNoise() {
    std::random_device rd;

    // random_device 를 통해 난수 생성 엔진을 초기화 한다.
    std::mt19937 gen(rd());

    // -100 부터 100 까지 균등하게 나타나는 난수열을 생성하기 위해 균등 분포 정의.
    std::uniform_int_distribution<int> dis(-RANDOM_BOUNDARY, RANDOM_BOUNDARY);
    double noisePosition = double(dis(gen)) / RANDOM_BOUNDARY * MAX_POSITION_NOISE;
    double noiseVelocity = double(dis(gen)) / RANDOM_BOUNDARY * MAX_VELOCITY_NOISE;

    mPosition[0] += noisePosition;
    mVelocity[0] += noiseVelocity;
}

void MIPPDController::doControl() {
    updateState();
    generateExternalForce();
    computeControlInput();
    setControlInput();
}

void MIPPDController::setTrajectory() {
    mDesiredRodPosition = 0.0;
    mDesiredRodVelocity = 0.0;
}

void MIPPDController::updateState() {
    mPosition = getRobot()->robot->getGeneralizedCoordinate();
    mVelocity = getRobot()->robot->getGeneralizedVelocity();

    addNoise();
}

void MIPPDController::computeControlInput() {
    mPositionError = mDesiredRodPosition - mPosition[0];
    mVelocityError = mDesiredRodVelocity - mVelocity[0];

    mTorque[1] = -mPGain * mPositionError - mDGain * mVelocityError;

    if(mTorque[1] > mTorqueLimit)
    {
        mTorque[1] = mTorqueLimit;
    }
    else if(mTorque[1] < -mTorqueLimit)
    {
        mTorque[1] = -mTorqueLimit;
    }
}

void MIPPDController::setControlInput() {
 getRobot()->robot->setGeneralizedForce(mTorque);
}

const Eigen::VectorXd &MIPPDController::getTorque() const {
    return mTorque;
}
