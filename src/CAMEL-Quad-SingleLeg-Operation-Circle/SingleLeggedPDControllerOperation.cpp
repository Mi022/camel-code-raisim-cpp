//
// Created by jaehoon on 22. 5. 2.
//

#include "SingleLeggedPDControllerOperation.h"

void SingleLeggedPDControllerOperation::setPDGain(double PGain, double DGain) {
    this->PGain = PGain;
    this->DGain = DGain;
}

void SingleLeggedPDControllerOperation::doControl() {
    updateState();
    setTrajectory();
    computeControlInput();
    setControlInput();
}

void SingleLeggedPDControllerOperation::setTrajectory() {
    if(mIsZeroing)
    {
        if(*mCurrentTime < mHaltTime)
        {
            desiredJointPosition << mCubicTrajectoryGen1.getPositionTrajectory(*mCurrentTime), mCubicTrajectoryGen2.getPositionTrajectory(*mCurrentTime);
            desiredJointVelocity << mCubicTrajectoryGen1.getVelocityTrajectory(*mCurrentTime), mCubicTrajectoryGen2.getVelocityTrajectory(*mCurrentTime);
        }
        else
        {
            mIsZeroing = false;
            setPointTrajectoryZeroing();
        }
    }
    else if(mIsCubic)
    {
        if(*mCurrentTime < mHaltTime)
        {
            desiredPosition = mCubicTrajectoryGen.getPositionTrajectory(*mCurrentTime);
            desiredVelocity = mCubicTrajectoryGen.getVelocityTrajectory(*mCurrentTime);
            IKsolve();
            std::cout<<mHaltTime<<std::endl;
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

void SingleLeggedPDControllerOperation::setPointTrajectory(double goalPosition){
    desiredPosition = goalPosition;
    desiredJointPosition[0] = acos(desiredPosition / 0.46);
    desiredJointPosition[1] = -2*desiredJointPosition[0];
    desiredJointPosition_past = desiredJointPosition;
    desiredJointVelocity.setZero();
}

void SingleLeggedPDControllerOperation::zeroing() {
    mIsZeroing = true;
    mIsCubic = false;
    mIsSin = false;
    updateState();
    double timeDuration = 3.0;
    mCubicTrajectoryGen1.updateTrajectory(position[1], 0.785398, *mCurrentTime, timeDuration);
    mCubicTrajectoryGen2.updateTrajectory(position[2], -1.570796, *mCurrentTime, timeDuration);
    mHaltTime = *mCurrentTime + timeDuration;
    std::cout<<"zeroing end"<<std::endl;
}

void SingleLeggedPDControllerOperation::setPointTrajectoryZeroing() {
    desiredJointPosition << 0.785398, -1.570796;
    desiredJointVelocity << 0.0, 0.0;
    desiredPosition = 0.325269119;
    desiredVelocity = 0.0;
}

void SingleLeggedPDControllerOperation::updateCubicTrajectory(double goalPosition, double timeDuration) {
    mIsZeroing = false;
    mIsCubic = true;
    mIsSin = false;
    mCubicTrajectoryGen.updateTrajectory(desiredPosition, goalPosition, *mCurrentTime, timeDuration);
    mHaltTime = *mCurrentTime + timeDuration;
}

void SingleLeggedPDControllerOperation::updateSinTrajectory(double amplitude, double frequency, double timeDuration) {
    mIsZeroing = false;
    mIsCubic = false;
    mIsSin = true;
}

void SingleLeggedPDControllerOperation::updateState() {
    position = mRobot->getQ();
    velocity = mRobot->getQD();
}

void SingleLeggedPDControllerOperation::computeControlInput() {
    for (int i = 1; i < 3; i++) {
        jointPositionError[i - 1] = desiredJointPosition[i - 1] - position[i];
        jointVelocityError[i - 1] = desiredJointVelocity[i - 1] - velocity[i];
        torque[i-1] = PGain * jointPositionError[i - 1] + DGain * jointVelocityError[i - 1];
    }
}

void SingleLeggedPDControllerOperation::setControlInput() {
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

void SingleLeggedPDControllerOperation::IKsolve() {
    desiredJointPosition_past = desiredJointPosition;
    desiredJointPosition[0] = acos(desiredPosition / 0.46);
    desiredJointPosition[1] = -2*desiredJointPosition[0];
    desiredJointVelocity = (desiredJointPosition - desiredJointPosition_past) / mDT;
}
