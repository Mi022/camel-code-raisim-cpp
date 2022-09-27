//
// Created by hwayoung on 22. 9. 26.
//

#include "DoubleBarLQRController.hpp"

const Eigen::MatrixXd& DoubleBarLQRController::GetX() const
{
    return mX;
}

void DoubleBarLQRController::doControl()
{
    updateState();
    if(mIsSExist){
        computeControlInput();
        setControlInput();
    }
}

void DoubleBarLQRController::setTrajectory()
{

}

void DoubleBarLQRController::updateState()
{
    mX.block(0,0,mDesiredPosition.size(), 1) = getRobot()->getQ().e();
    mX.block(mDesiredPosition.size(),0,mDesiredVelocity.size(), 1) = getRobot()->getQD().e();
}

void DoubleBarLQRController::computeControlInput()
{
    //X_bar
    mX.block(0,0,mDesiredPosition.size(), 1) -= mDesiredPosition;
    mX.block(mDesiredPosition.size(),0,mDesiredVelocity.size(), 1) -= mDesiredVelocity;

    //optimal U_bar
    mTorque.block(1,0,mTorque.size()-1, 1) = -mK*mX;
    std::cout<<"mTorque: "<<std::endl<<mTorque<<std::endl;
    //optimal U
    mTorque += mDesiredTorque;
}

void DoubleBarLQRController::setControlInput()
{
    getRobot()->robot->setGeneralizedForce(mTorque);
}

void DoubleBarLQRController::setSNGain(Eigen::VectorXd D)
{
    mSN = Eigen::MatrixXd(mX.size(), mX.size());
    mS = Eigen::MatrixXd(mX.size(), mX.size());
    mSN.setZero();
    mS.setZero();
    mSN.diagonal() = D;
}

void DoubleBarLQRController::setQGain(Eigen::VectorXd D)
{
    mQ = Eigen::MatrixXd(mX.size(), mX.size());
    mQ.setZero();
    mQ.diagonal() = D;
}

void DoubleBarLQRController::setRGain(Eigen::VectorXd D)
{
    mR = Eigen::MatrixXd(getRobot()->robot->getDOF() - 1, getRobot()->robot->getDOF() - 1);
    mR.setZero();
    mR.diagonal() = D;
    std::cout << "mR: " << std::endl << mR << std::endl;
}

void DoubleBarLQRController::findS()
{
    int iteration = 0;
    Eigen::MatrixXd sNext = mSN;
    Eigen::MatrixXd temp = mB.transpose() * sNext * mB + mR;
    mS = mA.transpose() * (sNext - sNext * mB * temp.inverse() * mB.transpose() * sNext) * mA + mQ;
    while (!bIsSEnough(mS, sNext))
    {
        iteration++;
        if (iteration > mMaximumIteration)
        {
            mIsSExist = false;
            break;
        }
        sNext = mS;
        mS = mA.transpose() * (sNext - sNext * mB * temp.inverse() * mB.transpose() * sNext) * mA + mQ;
    }
}

void DoubleBarLQRController::findK()
{
    Eigen::MatrixXd temp = mB.transpose() * mS * mB + mR;
    std::cout<<"mK : "<<std::endl<<mB.transpose()*mS*mA*temp.inverse()<<std::endl;
    mK = temp.inverse()*mB.transpose()*mS*mA;
}

bool DoubleBarLQRController::bIsSEnough(Eigen::MatrixXd S, Eigen::MatrixXd Snext)
{
    for (int row = 0; row < S.rows(); row++)
    {
        for (int col = 0; col < S.cols(); col++)
        {
            if (abs(S(row, col) - Snext(row, col)) >= mTolerance)
            {
                return false;
            }
        }
    }
    return true;
}

void DoubleBarLQRController::torqueLimit()
{
    for (int idx = 1; idx < 2; idx++)
    {
        if (mTorque[idx] > mTorqueLimit)
        {
            mTorque[idx] = mTorqueLimit;
        }
        else if (mTorque[idx] < -mTorqueLimit)
        {
            mTorque[idx] = -mTorqueLimit;
        }
    }
}

const int DoubleBarLQRController::mMaximumIteration = 10000;
const double DoubleBarLQRController::mTolerance = 0.001;