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
        case STATE_MOTOR_OFF:
        {
            mCan->turnOffMotor();
            sharedMemory->controlState = STATE_CONTROL_STOP;
            break;
        }
        case STATE_READY:
        {
            mCan->readMotorErrorStatus();
            for(int index = 0; index < MOTOR_NUM; index++)
            {
                mTorque[index] = 0;
            }
            setControlInput();
            break;
        }
        case STATE_HOME_READY:
        {
            mCubicTrajectoryGen[0].updateTrajectory(sharedMemory->motorPosition[HIP_IDX], 45.0 * D2R, sharedMemory->localTime, 1.0);
            mCubicTrajectoryGen[1].updateTrajectory(sharedMemory->motorPosition[KNEE_IDX], -90.0 * D2R, sharedMemory->localTime, 1.0);
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
//            doPDControl();
            break;
        }
        default:
            break;
    }
}

void JointPDController::setPDGain(double *Kp, double *Kd)
{
    for (int index = 0; index < MOTOR_NUM; index++)
    {
        this->Kp[index] = Kp[index];
        this->Kd[index] = Kd[index];
    }
}

void JointPDController::setTrajectory()
{
//    mDesiredPosition = 3.141592;
//    mDesiredVelocity = 0.0;
}

void JointPDController::computeControlInput()
{
    for (int index = 0; index < MOTOR_NUM; index++)
    {
        mTorque[index] = Kp[index] * (mDesiredPosition[index] - sharedMemory->motorPosition[index])
                       + Kd[index] * (mDesiredVelocity[index] - sharedMemory->motorVelocity[index]);
    }
}

void JointPDController::setControlInput()
{
    for (int index = 0; index < MOTOR_NUM; index++)
    {
        if (mTorque[index] > mTorqueLimit[index])
        {
            mTorque[index] = mTorqueLimit[index];
        }
        else if (mTorque[index] < -mTorqueLimit[index])
        {
            mTorque[index] = -mTorqueLimit[index];
        }
    }
    mCan->setTorque(mTorque);
}

void JointPDController::doHomeControl()
{
    for(int index = 0; index < MOTOR_NUM; index++)
    {
        mTorque[index] =
                Kp[index] * (mCubicTrajectoryGen[index].getPositionTrajectory(sharedMemory->localTime) - sharedMemory->motorPosition[index])
                + Kd[index] * (mCubicTrajectoryGen[index].getVelocityTrajectory(sharedMemory->localTime) - sharedMemory->motorVelocity[index]);
    }
    setControlInput();
}

void JointPDController::doPDControl()
{
    setTrajectory();
    computeControlInput();
    setControlInput();
}