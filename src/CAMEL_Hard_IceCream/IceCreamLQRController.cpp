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
            30.0552, -39.0838, 9.42587e-10, 4.23916e-10,
            -36.699,  117.033,  -2.49571e-09,  -9.42587e-10;

    Bc <<   0,
            0,
            -8.53125,
            22.5884;

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
    updateState();
    if(mIsSExist){
        computeControlInput();
        setControlInput();
    }
}

void IceCreamLQRController::setTrajectory() {

}

void IceCreamLQRController::updateState() {
    position = getRobot()->getQ().e();
    velocity = getRobot()->getQD().e();

    mX[0] = position[0];
    mX[1] = position[1] - desiredPosition[1];
    mX[2] = velocity[0];
    mX[3] = velocity[1];

}

void IceCreamLQRController::computeControlInput() {
    torque[1] = -mK*mX;
    torqueLimit();
}

void IceCreamLQRController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(torque);
}

void IceCreamLQRController::torqueLimit(){
    if(torque[1] > mTorqueLimit)
    {
        torque[1] = mTorqueLimit;
    }
    else if(torque[1] < -mTorqueLimit)
    {
        torque[1] = -mTorqueLimit;
    }
}

void IceCreamLQRController::raisimDynamics() {
        std::cout<<"mass matrix: "<<std::endl<<getRobot()->robot->getMassMatrix()<<std::endl;
        std::cout<<"inverse mass matrix: "<<std::endl<<getRobot()->robot->getInverseMassMatrix()<<std::endl;
        std::cout<<"nonlinear matrix: "<<std::endl<<getRobot()->robot->getNonlinearities({9.81, 0.0, 0.0})<<std::endl;
        std::cout<<"square: "<<std::endl<<getRobot()->robot->getInverseMassMatrix().e()*getRobot()->robot->getNonlinearities({9.81, 0.0, 0.0}).e()<<std::endl;

}
const int IceCreamLQRController::mMaximumIteration = 10000;
const double IceCreamLQRController::mTolerance = 0.001;