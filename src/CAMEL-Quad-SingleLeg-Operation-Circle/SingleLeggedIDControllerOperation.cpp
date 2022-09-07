//
// Created by jaehoon on 22. 6. 9.
//

#include "SingleLeggedIDControllerOperation.h"

void SingleLeggedIDControllerOperation::setPDGain_ID(double PGain_ID, double DGain_ID) {
    this->PGain_ID = PGain_ID;
    this->DGain_ID = DGain_ID;
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
//            desiredPosition = mQuinticTrajectoryGen.getPositionTrajectory(*mCurrentTime);
//            desiredVelocity = mQuinticTrajectoryGen.getVelocityTrajectory(*mCurrentTime);
//            desiredAcceleration = mQuinticTrajectoryGen.getAccelerationTrajectory(*mCurrentTime);

//            desiredJointPosition_past_PD = desiredJointPosition_PD;
//            desiredJointPosition_PD[0] = acos(desiredPosition / 0.46);
//            desiredJointPosition_PD[1] = -2*desiredJointPosition_PD[0];
//            desiredJointVelocity_PD = (desiredJointPosition_PD - desiredJointPosition_past_PD) / mDT;
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
        if(*mCurrentTime < mHaltTime)
        {
            desiredPosition = mQuinticTrajectoryGen.getPositionTrajectory(*mCurrentTime);
            desiredVelocity = mQuinticTrajectoryGen.getVelocityTrajectory(*mCurrentTime);
            desiredAcceleration = mQuinticTrajectoryGen.getAccelerationTrajectory(*mCurrentTime);
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

void SingleLeggedIDControllerOperation::setPointTrajectoryZeroing() {
    desiredJointPosition_PD << 0.785398, -1.570796;
    desiredJointVelocity_PD << 0.0, 0.0;
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
    if(mIsPDControl){
        computeControlInput_PD();
    }
    else if(mIsSwitchingController){
        computeControlInput_switch();
    }
    else{
        computeControlInput_ID();
    }
}

void SingleLeggedIDControllerOperation::computeControlInput_ID() {
    calculatedForce = mLumpedMass * (desiredAcceleration + PGain_ID * (desiredPosition - position[0]) + DGain_ID * (desiredVelocity - velocity[0]) - mGravity);
    dz_dth1 = -0.23*sin(position[1]) - 0.23*sin(position[1] + position[2]);
    dz_dth2 = -0.23*sin(position[1] + position[2]);
    torque[0] = dz_dth1 * calculatedForce;
    torque[1] = dz_dth2 * calculatedForce;
//    std::cout<<"input torque: "<<torque[0]<<" "<<torque[1]<<std::endl;
}

void SingleLeggedIDControllerOperation::computeControlInput_switch() {
    double portion = (*mCurrentTime - mHaltTime) / 3.0;
    computeControlInput_ID();
    torque *= portion;
    for (int i = 1; i < 3; i++) {
        torque[i-1] += (1.0 - portion)*(PGain_PD * (desiredJointPosition_PD[i - 1] - position[i]) + DGain_PD * (desiredJointVelocity_PD[i - 1] - velocity[i]));
    }
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

void SingleLeggedIDControllerOperation::setPDGain_PD(double PGain_PD, double DGain_PD) {
    this->PGain_PD = PGain_PD;
    this->DGain_PD = DGain_PD;
}

void SingleLeggedIDControllerOperation::computeControlInput_PD() {
    for (int i = 1; i < 3; i++) {
        torque[i-1] = PGain_PD * (desiredJointPosition_PD[i - 1] - position[i]) + DGain_PD * (desiredJointVelocity_PD[i - 1] - velocity[i]);
    }
}