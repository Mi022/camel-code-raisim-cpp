//
// Created by jaehoon on 22. 7. 28.
//

#include "SingleLeggedMPCOperation.h"


void SingleLeggedMPCOperation::doControl() {
    updateState();
    setTrajectory();
    computeControlInput();
    setControlInput();
    resetMPCVariables();
}

void SingleLeggedMPCOperation::setTrajectory() {
    if(mIsZeroing)
    {
        if(*mCurrentTime < mHaltTime)
        {
            desiredJointPosition_PD << mCubicTrajectoryGen_hip.getPositionTrajectory(*mCurrentTime), mCubicTrajectoryGen_knee.getPositionTrajectory(*mCurrentTime);
            desiredJointVelocity_PD << mCubicTrajectoryGen_hip.getVelocityTrajectory(*mCurrentTime), mCubicTrajectoryGen_knee.getVelocityTrajectory(*mCurrentTime);
        }
        else if((*mCurrentTime >= mHaltTime) && (*mCurrentTime < mHaltTime + 3.0))
        {
            mIsPDControl = false;
            mIsSwitchingController = true;
            setPointTrajectoryZeroing();
        }
        else
        {
            mIsZeroing = false;
            mIsSwitchingController = false;
//            setPointTrajectoryZeroing();
        }
    }
    else if(mIsCubic)
    {
        if(*mCurrentTime < (mHaltTime - mDT*mMPCHorizon))
        {
            for(int i = 0; i < mMPCHorizon ; i++)
            {
                mTrajectorySequence(0,i) = mQuinticTrajectoryGen.getPositionTrajectory(*mCurrentTime + mDT * i);
                mTrajectorySequence(1,i) = mQuinticTrajectoryGen.getVelocityTrajectory(*mCurrentTime + mDT * i);
            }
//    std::cout<<"mTrajectorySequence : \n"<<mTrajectorySequence<<std::endl;
            desiredPosition = mTrajectorySequence(0,0);
            desiredVelocity = mTrajectorySequence(1,0);
        }
        else if((*mCurrentTime >= (mHaltTime - mDT*mMPCHorizon)) && (*mCurrentTime < mHaltTime))
        {
            //TODO : Errors are in here
            int startIdx = (*mCurrentTime - mHaltTime + mDT*mMPCHorizon)/mDT;
            std::cout<<"startIdx : "<<startIdx<<std::endl;
            for(int i = 0; i < startIdx ; i++)
            {
                mTrajectorySequence(0,i) = mQuinticTrajectoryGen.getPositionTrajectory(*mCurrentTime + mDT * i);
                mTrajectorySequence(1,i) = mQuinticTrajectoryGen.getVelocityTrajectory(*mCurrentTime + mDT * i);
            }

            for(int i = startIdx; i < mMPCHorizon ; i++)
            {
                mTrajectorySequence(0,i) = mQuinticTrajectoryGen.getPositionTrajectory(*mCurrentTime + mDT * startIdx);
                mTrajectorySequence(1,i) = 0.0;
            }
            desiredPosition = mTrajectorySequence(0,0);
            desiredVelocity = mTrajectorySequence(1,0);
        }
        else
        {
            mIsCubic = false;
            setPointTrajectory(desiredPosition);
        }
    }
    else if(mIsSin)
    {

    }
}

void SingleLeggedMPCOperation::setPointTrajectory(double goalPosition){
    for(int i = 0; i < mMPCHorizon ; i++)
    {
        mTrajectorySequence(0,i) = goalPosition;
        mTrajectorySequence(1,i) = 0.0;
    }
    desiredPosition = goalPosition;
    desiredVelocity = 0.0;
}

void SingleLeggedMPCOperation::zeroing() {
    mIsPDControl = true;
    mIsZeroing = true;
    mIsCubic = false;
    mIsSin = false;
    updateState();
    double timeDuration = 3.0;
    desiredJointPosition_PD << position[1], position[2];
    mQuinticTrajectoryGen.updateTrajectory(position[0], 0.325269119, *mCurrentTime, timeDuration);
    mCubicTrajectoryGen_hip.updateTrajectory(position[1], 0.785398, *mCurrentTime, timeDuration);
    mCubicTrajectoryGen_knee.updateTrajectory(position[2], -1.570796, *mCurrentTime, timeDuration);
    mHaltTime = *mCurrentTime + timeDuration;
}

void SingleLeggedMPCOperation::setPointTrajectoryZeroing() {
    for(int i = 0; i < mMPCHorizon ; i++)
    {
        mTrajectorySequence(0,i) = 0.32526911;
        mTrajectorySequence(1,i) = 0.0;
    }
    desiredJointPosition_PD << 0.785398, -1.570796;
    desiredJointVelocity_PD << 0.0, 0.0;
    desiredPosition = 0.325269119;
    desiredVelocity = 0.0;
}

void SingleLeggedMPCOperation::updateCubicTrajectory(double goalPosition, double timeDuration) {
    mIsZeroing = false;
    mIsCubic = true;
    mIsSin = false;
    mQuinticTrajectoryGen.updateTrajectory(desiredPosition, goalPosition, *mCurrentTime, timeDuration);
    mHaltTime = *mCurrentTime + timeDuration;
}

void SingleLeggedMPCOperation::updateSinTrajectory(double amplitude, double frequency, double timeDuration) {
    mIsZeroing = false;
    mIsCubic = false;
    mIsSin = true;
}

void SingleLeggedMPCOperation::updateState() {
    position = mRobot->getQ();
    velocity = mRobot->getQD();
    mInitialPosition = position[0];
    mInitialVelocity = velocity[0];
}

void SingleLeggedMPCOperation::computeControlInput() {
    if(mIsPDControl){
        computeControlInput_PD();
    }
    else if(mIsSwitchingController){
        computeControlInput_switch();
    }
    else{
        computeControlInput_MPC();
    }
}

void SingleLeggedMPCOperation::computeControlInput_MPC() {
    solve();
    if(mForce(0) > mForceLimit)
    {
        calculatedForce = mForceLimit;
    }
    else if(mForce(0) < -mForceLimit)
    {
        calculatedForce = -mForceLimit;
    }
    else
    {
        calculatedForce = mForce(0);
    }

    dz_dth1 = -0.23*sin(position[1]) - 0.23*sin(position[1] + position[2]);
    dz_dth2 = -0.23*sin(position[1] + position[2]);
    torque[0] = dz_dth1 * calculatedForce;
    torque[1] = dz_dth2 * calculatedForce;
//    std::cout<<"calculated force: "<<calculatedForce<<std::endl;
}

void SingleLeggedMPCOperation::computeControlInput_switch() {
    double portion = (*mCurrentTime - mHaltTime) / 3.0;
    computeControlInput_MPC();
    torque *= portion;
    for (int i = 1; i < 3; i++) {
        torque[i-1] += (1.0 - portion)*(PGain_PD * (desiredJointPosition_PD[i - 1] - position[i]) + DGain_PD * (desiredJointVelocity_PD[i - 1] - velocity[i]));
    }
}
void SingleLeggedMPCOperation::setControlInput() {
    for (int i = 0; i < 2; i++) {
        if(torque[i] > mTorqueLimit)
        {
            torque[i] = mTorqueLimit;
        }
        else if(torque[i] < -mTorqueLimit)
        {
            torque[i] = -mTorqueLimit;
        }
    }
    mRobot->setTorque(torque);
}

void SingleLeggedMPCOperation::setPDGain_PD(double PGain_PD, double DGain_PD) {
    this->PGain_PD = PGain_PD;
    this->DGain_PD = DGain_PD;
}

void SingleLeggedMPCOperation::computeControlInput_PD() {
    for (int i = 1; i < 3; i++) {
        torque[i-1] = PGain_PD * (desiredJointPosition_PD[i - 1] - position[i]) + DGain_PD * (desiredJointVelocity_PD[i - 1] - velocity[i]);
    }
}

void SingleLeggedMPCOperation::solve() {
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

void SingleLeggedMPCOperation::updateMPCStates() {
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

void SingleLeggedMPCOperation::updateMPCStatesTemp(Eigen::VectorXd force) {
    for(int i = 0; i<mMPCHorizon;i++){
        if(i == 0){
            mNextStatesTemp(0, i) = mInitialPosition + mDT * mInitialVelocity;
            mNextStatesTemp(1, i) = mInitialVelocity + mDT * (force(i) / mLumpedMass + mGravity);
        }
        else if (i == 1){
            mNextStatesTemp(0, i) = mNextStatesTemp(0, i-1) + mDT * mInitialVelocity + mDT * mDT * (force(i-1) / mLumpedMass + mGravity);
            mNextStatesTemp(1, i) = mInitialVelocity + mDT * (force(i) / mLumpedMass + mGravity);
        }
        else{
            mNextStatesTemp(0, i) = mNextStates(0, i-1) + mDT * mNextStatesTemp(1,i-2) + mDT * mDT * (force(i-1) / mLumpedMass + mGravity);
            mNextStatesTemp(1, i) = mInitialVelocity + mDT * (force(i) / mLumpedMass + mGravity);
        }
    }
}

double SingleLeggedMPCOperation::objectiveFunction(Eigen::VectorXd force) {
    mObjFunctionValue = 0.0;
    updateMPCStatesTemp(force);

    for (int i = 0 ; i < mMPCHorizon ; i++)
    {
        tempValue1 = mNextStatesTemp(0, i)-mTrajectorySequence(0, i);
        tempValue2 = mNextStatesTemp(1, i)-mTrajectorySequence(1, i);
        mObjFunctionValue += tempValue1*tempValue1*mQ(0,0);
        mObjFunctionValue += tempValue2*tempValue2*mQ(1,1);
        mObjFunctionValue += mForce(i)*mR(0)*mForce(i);
    }
    return mObjFunctionValue;
}

void SingleLeggedMPCOperation::computeGradient() {

    double functionValue = objectiveFunction(mForce);

    for(int i = 0; i < mMPCHorizon ; i++)
    {
        mForceTemp = mForce;
        mForceTemp(i) += mDelta;
        mGradient[i] =  (objectiveFunction(mForceTemp) - functionValue) / mDelta;
    }
    mRMSGradient = pow(mGradient.dot(mGradient),0.5);
}

void SingleLeggedMPCOperation::updateForces() {
    for(int i = 0; i < mMPCHorizon ; i++)
    {
        mForce(i) -= mStepSize * mGradient(i);
    }
}

bool SingleLeggedMPCOperation::isTerminateCondition() {
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

void SingleLeggedMPCOperation::resetMPCVariables() {
    mIteration = 0;
    /*check point 1*/
//    mForce.setOnes();
//    mForce = mForce * mLumpedMass * mGravity * (-1);
    mRMSGradient = 10;
}