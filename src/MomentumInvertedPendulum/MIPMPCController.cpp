//
// Created by user on 22. 6. 17.
//

#include "MIPMPCController.h"

void MIPMPCController::setQGain(double Q11, double Q22, double Q33) {
    mQ <<    Q11, 0.0, 0.0,
            0.0, Q22, 0.0,
            0.0, 0.0, Q33;
}

void MIPMPCController::setRGain(double R) {
    this -> mR = R;
}

void MIPMPCController::updateU(){
    mU = mU - STEP_SIZE*mDJ;
}

void MIPMPCController::computeDJ(){
    VectorXd Udelta = VectorXd(mMPCHorizen);
    for(int i = 0; i<mMPCHorizen; i++){
        Udelta = mU;
        Udelta[i] = mU[i] + DELTA;
        mDJ[i] = (computeJ(Udelta) - computeJ(mU))/DELTA;
    }
}

double MIPMPCController::computeJ(VectorXd U) {
    double J = 0;
    mX_temp = mX;
    for(int i = 0; i<mMPCHorizen; i++){
        stateSpaceEquation(U[i]);
        mX_bar = mX_temp;
        mX_bar[0] -= mDesirePosition;
        J += mX_temp.transpose()*mQ*mX_temp + U[i]*mR*U[i];
    }
    return J;
}

void MIPMPCController::stateSpaceEquation(double u) {
    double theta_0 = mX_temp[0];
    mX_temp[0] = theta_0 + mDT*mX_temp[1] + mDT*mDT/2*132.1654*sin(theta_0)  - mDT*mDT/0.0035*u;
    mX_temp[1] = 132.1654*mDT*sin(theta_0) + mX_temp[1] -mDT/0.001741*u;
    mX_temp[2] = -132.1654*mDT*sin(theta_0) + mX_temp[2] + mDT*4.8884e+03*u;
}

bool MIPMPCController::IsBreak(int i){
    double RMS = pow((mDJ.transpose()*mDJ)(0)/mMPCHorizen,0.5);
    if(RMS < TERMINATE_CONDITION){
        return false;
    }
    if(i> ITERATION){
        return false;
    }
    return true;
}

void MIPMPCController::doControl() {
    updateState();
    computeControlInput();
    setControlInput();
}

void MIPMPCController::setTrajectory() {
    mDesirePosition = 0.2;
}

void MIPMPCController::updateState() {
    mPosition = getRobot()->robot->getGeneralizedCoordinate();
    mVelocity = getRobot()->robot->getGeneralizedVelocity();

    mX[0] = mPosition[0];
    mX[1] = mVelocity[0];
    mX[2] = mVelocity[1];
}

void MIPMPCController::computeControlInput() {
    int i = 0;
    bool doing = true;
    while(doing)
    {
        computeDJ();
        updateU();
        doing = IsBreak(i);
        i++;
    }
    mTorque[1] = mU[0];

    if(mTorque[1] > mTorqueLimit)
    {
        mTorque[1] = mTorqueLimit;
    }
    else if(mTorque[1] < -mTorqueLimit)
    {
        mTorque[1] = -mTorqueLimit;
    }
}

void MIPMPCController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(mTorque);
    mU.setZero();
}

const VectorXd &MIPMPCController::getTorque() const {
    return mTorque;
}

void MIPMPCController::setTorqueLimit(double torqueLimit) {
    mTorqueLimit = torqueLimit;
}
