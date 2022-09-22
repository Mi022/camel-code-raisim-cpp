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
            mCubicTrajectoryGen.updateTrajectory(sharedMemory->motorPosition, 0.881691, sharedMemory->localTime, 3.0);
            sharedMemory->controlState = STATE_HOME_CONTROL;
            break;
        }
        case STATE_HOME_CONTROL:
        {
            mCan->readMotorErrorStatus();
            doHomeControl();
            break;
        }
        case STATE_PD_READY:
        {
            mBezierTrajectoryGen.updateTrajectory(sharedMemory->localTime, 1);
            sharedMemory->controlState = STATE_PD_CONTROL;
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
    mBezierTrajectoryGen.getPositionTrajectory(sharedMemory->localTime);
    mDesiredP[0] = mBezierTrajectoryGen.sumX;
    mDesiredP[1] = mBezierTrajectoryGen.sumZ;

    double d = sqrt(pow(mDesiredP[0],2)+pow(mDesiredP[1],2));
    double phi = acos(abs(mDesiredP[0])/ d);
    double psi = acos(pow(d,2)/(2*0.23*d));

    if (mDesiredP[0] < 0)
        mDesiredPosition = 1.57 - phi + psi;
    else if(mDesiredP[0] == 0)
        mDesiredPosition = psi;
    else
        mDesiredPosition = phi + psi - 1.57;

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