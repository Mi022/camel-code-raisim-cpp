//
// Created by pnuca on 2022-06-15.
//

#include "SingleLeggedMPCController.h"

void SingleLeggedMPCController::doControl() {
    std::cout<<"simTime : "<<getRobot()->getWorldTime()<<std::endl;
    updateState();
    setTrajectory();
    solve();
    computeControlInput();
    setControlInput();
    std::cout<<"calculated forces : \n"<<mForce<<std::endl;
    resetMPCVariables();
}

void SingleLeggedMPCController::setTrajectory() {
    double currentTime = getRobot()->getWorldTime();
    for(int i = 0; i < mMPCHorizon ; i++)
    {
        mTrajectorySequence(0,i) = mTrajectoryGenerator.getPositionTrajectory(currentTime + mDT * i);
        mTrajectorySequence(1,i) = mTrajectoryGenerator.getVelocityTrajectory(currentTime + mDT * i);
    }
//    std::cout<<"mTrajectorySequence : \n"<<mTrajectorySequence<<std::endl;
    desiredPosition = mTrajectorySequence(0,0);
    desiredVelocity = mTrajectorySequence(1,0);
}

void SingleLeggedMPCController::updateState() {
    position = getRobot()->getQ();
    velocity = getRobot()->getQD();
    mInitialPosition = position[0];
    mInitialVelocity = velocity[0];
    mForce(0) = mInitialForce;
}

void SingleLeggedMPCController::computeControlInput() {
    calculatedForce = mForce(0);
    dz_dth1 = -0.23*sin(position[1]) - 0.23*sin(position[1] + position[2]);
    dz_dth2 = -0.23*sin(position[1] + position[2]);
    torque[1] = dz_dth1 * calculatedForce;
    torque[2] = dz_dth2 * calculatedForce;
}

void SingleLeggedMPCController::setControlInput() {
    for (int i = 0; i < 3; i++) {
        if(torque[i] > torqueLimit)
        {
            torque[i] = torqueLimit;
        }
        else if(torque[i] < -torqueLimit)
        {
            torque[i] = -torqueLimit;
        }
    }
    getRobot()->robot->setGeneralizedForce(torque);
}

void SingleLeggedMPCController::solve() {
    while(true){
        mIteration++;
        updateMPCStates();
        computeGradient();
        updateForces();
        if(isTerminateCondition()){
            std::cout<<"iteration : "<<mIteration<<std::endl;
//            std::cout<<"mNextStates : \n"<<mNextStates<<"\n\n"<<std::endl;
            break;
        }
    }
}

void SingleLeggedMPCController::updateMPCStates() {
    for(int i = 0; i<mMPCHorizon;i++){
        if(i == 0){
            mNextStates(0, i) = mInitialPosition + mDT * mInitialVelocity;
            mNextStates(1, i) = mInitialVelocity + mDT * (mForce(i) / mLumpedMass + mGravity);
        }
        else if (i == 1){
            mNextStates(0, i) = mNextStates(0, i-1) + mDT * mInitialVelocity + mDT * mDT * (mForce(i-1)/mLumpedMass + mGravity);
            mNextStates(1, i) = mInitialVelocity + mDT * (mForce(i) / mLumpedMass + mGravity);
        }
        else{
            mNextStates(0, i) = mNextStates(0, i-1) + mDT * mNextStates(1,i-2) + mDT * mDT * (mForce(i-1)/mLumpedMass + mGravity);
            mNextStates(1, i) = mInitialVelocity + mDT * (mForce(i) / mLumpedMass + mGravity);
        }
    }
}

void SingleLeggedMPCController::updateMPCStatesTemp(Eigen::VectorXd force){
    for(int i = 0; i<mMPCHorizon;i++){
        if(i == 0){
            mNextStatesTemp(0, i) = mInitialPosition + mDT * mInitialVelocity;
            mNextStatesTemp(1, i) = mInitialVelocity + mDT * (force(i) / mLumpedMass + mGravity);
        }
        else if (i == 1){
            mNextStatesTemp(0, i) = mNextStatesTemp(0, i-1) + mDT * mInitialVelocity + mDT * mDT * (force(i-1)/mLumpedMass + mGravity);
            mNextStatesTemp(1, i) = mInitialVelocity + mDT * (force(i) / mLumpedMass + mGravity);
        }
        else{
            mNextStatesTemp(0, i) = mNextStates(0, i-1) + mDT * mNextStatesTemp(1,i-2) + mDT * mDT * (force(i-1)/mLumpedMass + mGravity);
            mNextStatesTemp(1, i) = mInitialVelocity + mDT * (force(i) / mLumpedMass + mGravity);
        }
    }
}

double SingleLeggedMPCController::objectiveFunction(Eigen::VectorXd force) {
    double ret = 0.0;
    updateMPCStatesTemp(force);
    Eigen::VectorXd nextX(2);
    Eigen::VectorXd nextXDes(2);
    for (int i = 0 ; i < mMPCHorizon ; i++)
    {
        nextX(0) = mNextStatesTemp(0, i);
        nextX(1) = mNextStatesTemp(1, i);
        nextXDes(0) = mTrajectorySequence(0, i);
        nextXDes(1) = mTrajectorySequence(1, i);
        ret += ((nextX - nextXDes).transpose()*mQ*(nextX - nextXDes) + mForce(i)*mR*mForce(i))(0);
    }
    return ret;
}

void SingleLeggedMPCController::computeGradient() {
    double functionValue = objectiveFunction(mForce);
    for(int i = 0; i < mMPCHorizon ; i++)
    {
        mForceTemp = mForce;
        mForceTemp(i) += mDelta;
        mGradient[i] =  (objectiveFunction(mForceTemp) - functionValue) / mDelta;
    }
    mRMSGradient = pow(mGradient.dot(mGradient),0.5);
}

void SingleLeggedMPCController::updateForces() {
    for(int i = 0; i < mMPCHorizon ; i++)
    {
        mForce(i) -= mStepSize * mGradient(i);
    }
}

bool SingleLeggedMPCController::isTerminateCondition() {
    if(mIteration > mMaximumIteration){
        std::cout<<"maximum iteration"<<std::endl;
        return true;
    }
    else if(mRMSGradient < mTerminateCondition){
        std::cout<<"terminate condition"<<std::endl;
        return true;
    }
    else
        return false;
}

void SingleLeggedMPCController::resetMPCVariables() {
    mIteration = 0;
    mForce.setOnes();
    mForce = mForce * mLumpedMass * mGravity * (-1);
    mRMSGradient = 10;
}