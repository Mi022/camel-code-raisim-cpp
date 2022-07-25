//
// Created by jaehoon on 22. 6. 9.
//

#include "SingleLeggedIDControllerOperation.h"

void SingleLeggedIDControllerOperation::setPDGain(double PGain, double DGain) {
    this->PGain = PGain;
    this->DGain = DGain;
}

void SingleLeggedIDControllerOperation::doControl() {
    updateState();
    setTrajectory();
    computeControlInput();
    setControlInput();
}

void SingleLeggedIDControllerOperation::setTrajectory() {
    if(mIsZeroing)
    {
        if(*mCurrentTime < mHaltTime)
        {
            desiredPosition = mQuinticTrajectoryGen.getPositionTrajectory(*mCurrentTime);
            desiredVelocity = mQuinticTrajectoryGen.getVelocityTrajectory(*mCurrentTime);
            desiredAcceleration = mQuinticTrajectoryGen.getAccelerationTrajectory(*mCurrentTime);
            std::cout<<"halt time: "<<mHaltTime<<std::endl;
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
            desiredPosition = mQuinticTrajectoryGen.getPositionTrajectory(*mCurrentTime);
            desiredVelocity = mQuinticTrajectoryGen.getVelocityTrajectory(*mCurrentTime);
            desiredAcceleration = mQuinticTrajectoryGen.getAccelerationTrajectory(*mCurrentTime);
            std::cout<<"halt time: "<<mHaltTime<<std::endl;
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

void SingleLeggedIDControllerOperation::setPointTrajectory(double goalPosition){
    desiredPosition = goalPosition;
    desiredVelocity = 0.0;
    desiredAcceleration = 0.0;
}

void SingleLeggedIDControllerOperation::zeroing() {
    mIsZeroing = true;
    mIsCubic = false;
    mIsSin = false;
    updateState();
    double timeDuration = 3.0;
    mQuinticTrajectoryGen.updateTrajectory(position[0], 0.325269119, *mCurrentTime, timeDuration);
    mHaltTime = *mCurrentTime + timeDuration;
}

void SingleLeggedIDControllerOperation::setPointTrajectoryZeroing() {
    desiredPosition = 0.325269119;
    desiredVelocity = 0.0;
    desiredAcceleration = 0.0;
}

void SingleLeggedIDControllerOperation::updateCubicTrajectory(double goalPosition, double timeDuration) {
    mIsZeroing = false;
    mIsCubic = true;
    mIsSin = false;
    mQuinticTrajectoryGen.updateTrajectory(desiredPosition, goalPosition, *mCurrentTime, timeDuration);
    mHaltTime = *mCurrentTime + timeDuration;
}

void SingleLeggedIDControllerOperation::updateSinTrajectory(double amplitude, double frequency, double timeDuration) {
    mIsZeroing = false;
    mIsCubic = false;
    mIsSin = true;
}

void SingleLeggedIDControllerOperation::updateState() {
    position = mRobot->getQ();
    velocity = mRobot->getQD();
}

void SingleLeggedIDControllerOperation::computeControlInput() {
    calculatedForce = mLumpedMass * (desiredAcceleration + PGain * (desiredPosition - position[0]) + DGain * (desiredVelocity - velocity[0]) - mGravity);
    dz_dth1 = -0.23*sin(position[1]) - 0.23*sin(position[1] + position[2]);
    dz_dth2 = -0.23*sin(position[1] + position[2]);
    torque[0] = dz_dth1 * calculatedForce;
    torque[1] = dz_dth2 * calculatedForce;
//    std::cout<<"input torque: "<<torque[0]<<" "<<torque[1]<<std::endl;
}

void SingleLeggedIDControllerOperation::setControlInput() {
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