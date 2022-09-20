//
// Created by user on 22. 6. 11.
// this controller is for balancing
// desired states (position1, velocity1, velocity2) are zeros
//

#include "MIPLQRController.h"

void MIPLQRController::setMatrix() {
    mA <<    1.00165, 0.005, 0.0,
            0.6606, 1.0, 0.0,
            -0.6606, 0.0, 1.0;
    mB <<    0.00718,
            -2.872,
            24.442;
}
void MIPLQRController::setSNGain(double SN11, double SN22, double SN33) {
    mSN<<    SN11, 0.0, 0.0,
            0.0, SN22, 0.0,
            0.0, 0.0, SN33;
}

void MIPLQRController::setQGain(double Q11, double Q22, double Q33) {
    mQ <<    Q11, 0.0, 0.0,
            0.0, Q22, 0.0,
            0.0, 0.0, Q33;
}

void MIPLQRController::setRGain(double R) {
    this -> mR = R;
}

void MIPLQRController::findS() {
    int iteration = 0;
    Eigen::Matrix3d snext = mSN;
    double temp = mB.transpose() * snext * mB + mR;
    mS = mA.transpose() * (snext - snext * mB / temp * mB.transpose() * snext) * mA + mQ;
    while(!IsSEnough(mS, snext)){
         iteration++;
         if(iteration > MAXIMUM_ITERATION)
         {
             mIsSExist = false;
             break;
         }
         snext = mS;
         mS = mA.transpose() * (snext - snext * mB / (mB.transpose() * snext * mB + mR) * mB.transpose() * snext) * mA + mQ;
    }
}

bool MIPLQRController::IsSEnough(Eigen::Matrix3d S, Eigen::Matrix3d Snext) {
    for(int r = 0; r<3; r++){
        for(int c = 0; c<3; c++){
            if(abs(S(r,c) - Snext(r, c)) >= TOLERANCE)  return false;
        }
    }
    return true;
}
void MIPLQRController::findK() {
    double temp = mB.transpose()*mS*mB + mR;
    mK = mB.transpose()*mS*mA/temp;
}

void MIPLQRController::generateExternalForce() {
    raisim::Vec<3> externalForce = {0.0, 1.9, 0.0};
    raisim::Vec<3> forcePosition = {0.0, 0.0, 0.08};

    if(mIteration%FORCE_DURATION == 0 || mIteration == 0){
        getRobot()->robot->setExternalForce(1, forcePosition, externalForce);
    }
    mIteration++;
}

void MIPLQRController::addNoise() {
    std::random_device rd;

    // random_device 를 통해 난수 생성 엔진을 초기화 한다.
    std::mt19937 gen(rd());

    // -100 부터 100 까지 균등하게 나타나는 난수열을 생성하기 위해 균등 분포 정의.
    std::uniform_int_distribution<int> dis(-RANDOM_BOUNDARY, RANDOM_BOUNDARY);
    double noisePosition = double(dis(gen)) / RANDOM_BOUNDARY * MAX_POSITION_NOISE; //0.15
    double noiseVelocity = double(dis(gen)) / RANDOM_BOUNDARY * MAX_VELOCITY_NOISE; //1.4
    double noiseMotorVelocity = double(dis(gen)) / RANDOM_BOUNDARY * MAX_MOTOR_VELOCITY_NOISE; //35.5

    mPosition[0] += noisePosition;
    mVelocity[0] += noiseVelocity;
    mVelocity[1] += noiseMotorVelocity;

    mX[0] = mPosition[0];
    mX[1] = mVelocity[0];
    mX[2] = mVelocity[1];
}

void MIPLQRController::doControl() {
    updateState();
    addNoise();
    generateExternalForce();
    if(mIsSExist)
    {
        computeControlInput();
        setControlInput();
    }
}

void MIPLQRController::updateState() {
    mPosition = getRobot()->getQ();
    mVelocity = getRobot()->getQD();

    mX[0] = mPosition[0];
    mX[1] = mVelocity[0];
    mX[2] = mVelocity[1];
}

void MIPLQRController::computeControlInput() {
    mTorque[1] = -mK*mX;
    if(mTorque[1] > mTorqueLimit)
    {
        mTorque[1] = mTorqueLimit;
    }
    else if(mTorque[1] < -mTorqueLimit)
    {
        mTorque[1] = -mTorqueLimit;
    }
}

void MIPLQRController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(mTorque);
}

void MIPLQRController::setTrajectory() {

}

const Eigen::VectorXd &MIPLQRController::getTorque() const {
    return mTorque;
}