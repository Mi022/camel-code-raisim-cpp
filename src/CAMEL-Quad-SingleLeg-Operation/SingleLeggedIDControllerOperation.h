//
// Created by jaehoon on 22. 6. 9.
//

#ifndef RAISIM_SINGLELEGGEDIDCONTROLLEROPERATION_H
#define RAISIM_SINGLELEGGEDIDCONTROLLEROPERATION_H

#include "SingleLeggedRobotOperation.h"
#include "include/TrajectoryGenerator/CubicTrajectoryGenerator.h"
#include "include/TrajectoryGenerator/QuinticTrajectoryGenerator.h"

class SingleLeggedIDControllerOperation {
public:
    Eigen::VectorXd torque = Eigen::VectorXd(2);
    raisim::VecDyn position = raisim::VecDyn(3);
    raisim::VecDyn velocity = raisim::VecDyn(3);

    double desiredPosition = 0.0;
    double desiredVelocity = 0.0;
    double desiredAcceleration = 0.0;
    double calculatedForce = 0.0;

    // jacobian
    double dz_dth1 = 0.0;
    double dz_dth2 = 0.0;

    double PGain;
    double DGain;



    SingleLeggedIDControllerOperation(SingleLeggedRobotOperation *robot, double *currentTime, double dT) {
        mRobot = robot;
        setPDGain(70.0, 2.5);
        torque[0] = 0.0;
        mCurrentTime = currentTime;
        mDT = dT;
    }

    void doControl();
    void setTrajectory();
    void setPointTrajectory(double goalPosition);
    void zeroing();
    void updateCubicTrajectory(double goalPosition, double timeDuration);
    void updateSinTrajectory(double amplitude, double frequency, double timeDuration);
    void setPointTrajectoryZeroing();
    void updateState();
    void computeControlInput();
    void setControlInput();
    void setPDGain(double PGain, double DGain);

private:
    SingleLeggedRobotOperation *mRobot;
    QuinticTrajectoryGenerator mQuinticTrajectoryGen;
    bool mIsZeroing = false;
    bool mIsCubic = false;
    bool mIsSin = false;
    double mTorqueLimit = 10.0;
    double mHaltTime = 0.0;
    double *mCurrentTime;
    double mLumpedMass = 2.766;
    double mGravity = -9.81;
    double mDT;
};


#endif //RAISIM_SINGLELEGGEDIDCONTROLLEROPERATION_H
