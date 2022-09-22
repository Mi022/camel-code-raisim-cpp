//
// Created by camel on 22. 9. 21.
//

#ifndef RAISIM_JOINTPDCONTROLLER_H
#define RAISIM_JOINTPDCONTROLLER_H

#include "include/TrajectoryGenerator/CubicTrajectoryGenerator.h"
#include "include/TrajectoryGenerator/BezierTrajectoryGenerator.h"
<<<<<<< HEAD
#include "../Robot/RobotDescription.h"
=======
>>>>>>> 51bb5117ac89922faadf7256bca56ced31c74b94
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
<<<<<<< HEAD
    CubicTrajectoryGenerator mCubicTrajectoryGen[MOTOR_NUM];
    BezierTrajectoryGenerator mBezierTrajectoryGen;

    MotorCAN *mCan;
    int mIteration;
    double mDesiredP[MOTOR_NUM] = {-0.125, -0.37};
    double mDesiredPosition[MOTOR_NUM];
    double mDesiredVelocity[MOTOR_NUM];
    double Kp[MOTOR_NUM];
    double Kd[MOTOR_NUM];
    double mTorque[MOTOR_NUM];
    double mTorqueLimit[MOTOR_NUM];
=======
    CubicTrajectoryGenerator mCubicTrajectoryGen;
    BezierTrajectoryGenerator mBezierTrajectoryGen;
    MotorCAN *mCan;
    int mIteration;
    double mDesiredP[2] = {-0.125,-0.37};
    double mDesiredPosition;
    double mDesiredVelocity;
    double Kp;
    double Kd;
    double mTorque;
    double mTorqueLimit;
>>>>>>> 51bb5117ac89922faadf7256bca56ced31c74b94

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
