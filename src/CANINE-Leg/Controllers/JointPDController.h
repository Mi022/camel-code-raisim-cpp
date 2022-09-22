//
// Created by camel on 22. 9. 21.
//

#ifndef RAISIM_JOINTPDCONTROLLER_H
#define RAISIM_JOINTPDCONTROLLER_H

#include "include/TrajectoryGenerator/CubicTrajectoryGenerator.h"
#include "../Robot/RobotDescription.h"
#include "../Utils/SharedMemory.h"
#include "../Utils/MotorCAN.h"

class JointPDController
{
public:
    JointPDController(MotorCAN *can) :
    mCan(can)
    {
        mIteration = 0;
        Kp[HIP_IDX] = 30.0;
        Kd[HIP_IDX] = 1.5;
        Kp[KNEE_IDX] = 30.0;
        Kd[KNEE_IDX] = 1.5;
        mTorqueLimit[HIP_IDX] = 3.0;
        mTorqueLimit[KNEE_IDX] = 3.0;
    }

private:
    CubicTrajectoryGenerator mCubicTrajectoryGen[MOTOR_NUM];

    MotorCAN *mCan;
    int mIteration;
    double mDesiredPosition[MOTOR_NUM];
    double mDesiredVelocity[MOTOR_NUM];
    double Kp[MOTOR_NUM];
    double Kd[MOTOR_NUM];
    double mTorque[MOTOR_NUM];
    double mTorqueLimit[MOTOR_NUM];

public:
    void controllerFunction();

private:
    void setPDGain(double Kp[MOTOR_NUM], double Kd[MOTOR_NUM]);
    void setTrajectory();
    void computeControlInput();
    void setControlInput();
    void doHomeControl();
    void doPDControl();
};



#endif //RAISIM_JOINTPDCONTROLLER_H
