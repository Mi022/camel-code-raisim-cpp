//
// Created by jaehoon on 22. 4. 3..
//

#ifndef RAISIM_SIMPLEPENDULUMPDCONTROLLER_H
#define RAISIM_SIMPLEPENDULUMPDCONTROLLER_H

#include "SingleLeggedRobotOperation.h"
#include "include/TrajectoryGenerator/CubicTrajectoryGenerator.h"

class SingleLeggedPDControllerOperation {
public:
    Eigen::VectorXd torque = Eigen::VectorXd(2);
    raisim::VecDyn position = raisim::VecDyn(3);
    raisim::VecDyn velocity = raisim::VecDyn(3);
    Eigen::VectorXd jointPositionError = Eigen::VectorXd(2);
    Eigen::VectorXd jointVelocityError = Eigen::VectorXd(2);
    Eigen::VectorXd desiredJointPosition = Eigen::VectorXd(2);
    Eigen::VectorXd desiredJointPosition_past = Eigen::VectorXd(2);
    Eigen::VectorXd desiredJointVelocity = Eigen::VectorXd(2);

    double desiredPosition = 0.0;
    double desiredVelocity = 0.0;
    double positionError = 0.0;
    double velocityError = 0.0;

    double PGain;
    double DGain;

    SingleLeggedPDControllerOperation(SingleLeggedRobotOperation *robot, double *currentTime, double dT) {
        mRobot = robot;
        setPDGain(70.0, 0.5);
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
    void IKsolve();

private:
    SingleLeggedRobotOperation *mRobot;
    CubicTrajectoryGenerator mCubicTrajectoryGen;
    CubicTrajectoryGenerator mCubicTrajectoryGen1;
    CubicTrajectoryGenerator mCubicTrajectoryGen2;
    bool mIsZeroing = false;
    bool mIsCubic = false;
    bool mIsSin = false;
    double mTorqueLimit = 11.0;
    double mHaltTime = 0.0;
    double *mCurrentTime;
    double mDT;
};


#endif //RAISIM_SIMPLEPENDULUMPDCONTROLLER_H
