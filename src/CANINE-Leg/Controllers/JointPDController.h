//
// Created by camel on 22. 9. 21.
//

#ifndef RAISIM_JOINTPDCONTROLLER_H
#define RAISIM_JOINTPDCONTROLLER_H

#include "include/TrajectoryGenerator/CubicTrajectoryGenerator.h"
#include "include/TrajectoryGenerator/BezierTrajectoryGenerator.h"
#include "../Utils/SharedMemory.h"
#include "../Utils/MotorCAN.h"

class JointPDController
{
public:
    JointPDController(MotorCAN *can)
    {
        mCan = can;
        setPDGain(30.0, 1.5);
        mIteration = 0;
        mTorqueLimit = 3.0;
    }

private:
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

public:
    void controllerFunction();

private:
    void setPDGain(double Kp, double Kd);
    void setTrajectory();
    void computeControlInput();
    void setControlInput();
    void doHomeControl();
    void doPDControl();
};



#endif //RAISIM_JOINTPDCONTROLLER_H
