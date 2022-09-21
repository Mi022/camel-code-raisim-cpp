//
// Created by camel on 22. 9. 21.
//

#include "JointPDController.h"

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

void JointPDController::controllerFunction()
{
    sharedMemory->localTime = mIteration * CONTROL_dT;
    mIteration++;
    switch(sharedMemory->controlState)
    {
        case STATE_CONTROL_STOP:
        {
            break;
        }
        case STATE_READY:
        {
            mCan->readMotorErrorStatus();
            mCan->setTorque(0.0);
            break;
        }
        case STATE_HOME_READY:
        {
            mCubicTrajectoryGen.updateTrajectory(sharedMemory->motorPosition, 0.0, sharedMemory->localTime, 3.0);
            sharedMemory->controlState = STATE_HOME_CONTROL;
            break;
        }
        case STATE_HOME_CONTROL:
        {
            mCan->readMotorErrorStatus();
            doHomeControl();
            break;
        }
        case STATE_PD_CONTROL:
        {
            mCan->readMotorErrorStatus();
            doPDControl();
            break;
        }
        default:
            break;
    }
}

void JointPDController::setPDGain(double Kp, double Kd)
{
    this->Kp = Kp;
    this->Kd = Kd;
}

void JointPDController::setTrajectory()
{
    mDesiredPosition = 3.141592;
    mDesiredVelocity = 0.0;
}

void JointPDController::computeControlInput()
{
    mTorque = Kp * (mDesiredPosition - sharedMemory->motorPosition) + Kd * (mDesiredVelocity - sharedMemory->motorVelocity);
}

void JointPDController::setControlInput()
{
    if(mTorque > mTorqueLimit)
    {
        mTorque = mTorqueLimit;
    }
    else if(mTorque < -mTorqueLimit)
    {
        mTorque = -mTorqueLimit;
    }
    mCan->setTorque(mTorque);
}

void JointPDController::doHomeControl()
{
    mTorque = Kp * (mCubicTrajectoryGen.getPositionTrajectory(sharedMemory->localTime) - sharedMemory->motorPosition)
              + Kd * (mCubicTrajectoryGen.getVelocityTrajectory(sharedMemory->localTime) - sharedMemory->motorVelocity);
    setControlInput();
}

void JointPDController::doPDControl()
{
    setTrajectory();
    computeControlInput();
    setControlInput();
}