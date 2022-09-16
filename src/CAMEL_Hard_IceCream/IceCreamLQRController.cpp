//
// Created by hwayoung on 22. 9. 14.
//

#include "IceCreamLQRController.h"

void IceCreamLQRController::setMatrix() {
    Eigen::MatrixXd Ac, Bc, ABc;
    Ac = Eigen::MatrixXd(4, 4);
    Bc = Eigen::MatrixXd(4, 1);
    ABc = Eigen::MatrixXd(5, 5);

    Ac <<   0, 0, 1, 0,
            0, 0, 0, 1,
            52.1064, -39.7950, 0, 0,
            -43.0680, 53.2449, 0, 0;

    Bc <<   0,
            0,
            2.3899,
            -3.1977;

    ABc.setZero();

    ABc.block(0,0,4,4) = Ac;
    ABc.block(0,4,4,1) = Bc;
    ABc = ABc*mDT;
    ABc = ABc.exp(); // y'=My => y(t)=exp(M)y(0)
    mA = ABc.block(0,0,4,4);
    mB = ABc.block(0,4,4,1);
}

void IceCreamLQRController::setSNGain(Eigen::Vector4d D) {
    mSN.setZero();
    mSN(0, 0) = D(0);
    mSN(1, 1) = D(1);
    mSN(2, 2) = D(2);
    mSN(3, 3) = D(3);
}

void IceCreamLQRController::setQGain(Eigen::Vector4d D) {
    mQ.setZero();
    mQ(0, 0) = D(0);
    mQ(1, 1) = D(1);
    mQ(2, 2) = D(2);
    mQ(3, 3) = D(3);
}

void IceCreamLQRController::setRGain(double R) {
    this -> mR = R;
}

void IceCreamLQRController::findS() {
    int iteration = 0;
    Eigen::MatrixXd snext = mSN;
    double temp = mB.transpose() * snext * mB + mR;
    mS = mA.transpose() * (snext - snext * mB / temp * mB.transpose() * snext) * mA + mQ;
    while(!IsSEnough(mS, snext)){
        iteration++;
        if(iteration > mMaximumIteration)
        {
            mIsSExist = false;
            break;
        }
        snext = mS;
        mS = mA.transpose() * (snext - snext * mB / (mB.transpose() * snext * mB + mR) * mB.transpose() * snext) * mA + mQ;
    }
}

bool IceCreamLQRController::IsSEnough(Eigen::MatrixXd S, Eigen::MatrixXd Snext) {
    for(int r = 0; r<S.rows(); r++){
        for(int c = 0; c<S.cols(); c++){
            if(abs(S(r,c) - Snext(r, c)) >= mTolerance)  return false;
        }
    }
    return true;
}

void IceCreamLQRController::findK() {
    double temp = mB.transpose()*mS*mB + mR;
    mK = mB.transpose()*mS*mA/temp;
}

void IceCreamLQRController::doControl() {

}

void IceCreamLQRController::setTrajectory() {

}

void IceCreamLQRController::updateState() {

}

void IceCreamLQRController::computeControlInput() {

}

void IceCreamLQRController::setControlInput() {

}
const int IceCreamLQRController::mMaximumIteration = 10000;
const double IceCreamLQRController::mTolerance = 0.001;