//
// Created by jaehoon on 22. 7. 28.
//

#ifndef RAISIM_SINGLELEGGEDMPCOPERATION_H
#define RAISIM_SINGLELEGGEDMPCOPERATION_H

#include "SingleLeggedRobotOperation.h"
#include "include/TrajectoryGenerator/CubicTrajectoryGenerator.h"
#include "include/TrajectoryGenerator/QuinticTrajectoryGenerator.h"

class SingleLeggedMPCOperation {
public:
    Eigen::VectorXd torque = Eigen::VectorXd(2);
    raisim::VecDyn position = raisim::VecDyn(3);
    raisim::VecDyn velocity = raisim::VecDyn(3);

    double desiredPosition = 0.0;
    double desiredVelocity = 0.0;
    double calculatedForce = 0.0;

    // jacobian
    double dz_dth1 = 0.0;
    double dz_dth2 = 0.0;

    Eigen::VectorXd desiredJointPosition_PD = Eigen::VectorXd(2);
    Eigen::VectorXd desiredJointPosition_past_PD = Eigen::VectorXd(2);
    Eigen::VectorXd desiredJointVelocity_PD = Eigen::VectorXd(2);
    double PGain_PD;
    double DGain_PD;


    SingleLeggedMPCOperation(SingleLeggedRobotOperation *robot, double *currentTime, double dT) {
        mRobot = robot;
        setPDGain_PD(100.0, 0.5);
        torque.setZero();
        mCurrentTime = currentTime;
        mDT = dT;

        mMaximumIteration = 100;
        mTerminateCondition = 2.0 * 1e-6;
        mDelta = 1e-3;
        mStepSize = 80.0 / mDT / mDT;
        mA << 1.0, mDT, 0.0, 1.0;
        mB << 0.0, mDT/mLumpedMass;
        mC << 0.0, mGravity;
        mQ << 1.5, 0.0, 0.0, 1e-7;
        mR << mDT * mDT;
        mIteration = 0;
        mForce.setOnes();
        mForce = mForce * mLumpedMass * mGravity * (-1);
        calculatedForce = 0.0;
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
    void computeControlInput_MPC();
    void setControlInput();

    void setPDGain_PD(double PGain_PD, double DGain_PD);
    void computeControlInput_PD();
    void computeControlInput_switch();

    void solve();
    void updateMPCStates();
    void updateMPCStatesTemp(Eigen::VectorXd force);
    void computeGradient();
    void updateForces();
    void resetMPCVariables();
    bool isTerminateCondition();
    double objectiveFunction(Eigen::VectorXd force);

private:
    SingleLeggedRobotOperation *mRobot;
    QuinticTrajectoryGenerator mQuinticTrajectoryGen;
    CubicTrajectoryGenerator mCubicTrajectoryGen_hip;
    CubicTrajectoryGenerator mCubicTrajectoryGen_knee;
    bool mIsPDControl = false;
    bool mIsZeroing = false;
    bool mIsCubic = false;
    bool mIsSin = false;
    bool mIsSwitchingController = false;
    double mTorqueLimit = 10.0;
    double mForceLimit = 100.0;
    double mHaltTime = 0.0;
    double *mCurrentTime;
    double mLumpedMass = 2.766;
    double mGravity = -9.81;
    double mDT;

    int mMPCHorizon = 5;
    int mIteration;
    int mMaximumIteration;
    double mTerminateCondition;
    double mDelta;
    double mStepSize;
    double mInitialPosition;
    double mInitialVelocity;
//    double mInitialForce = 19.7083;
    double mRMSGradient;
    double mObjFunctionValue;
    double tempValue1;
    double tempValue2;
    Eigen::MatrixXd mTrajectorySequence = Eigen::MatrixXd(2, mMPCHorizon);
    Eigen::MatrixXd mA = Eigen::MatrixXd(2, 2);
    Eigen::VectorXd mB = Eigen::VectorXd(2);
    Eigen::VectorXd mC = Eigen::VectorXd(2);
    Eigen::MatrixXd mQ = Eigen::MatrixXd(2, 2);
    Eigen::VectorXd mR = Eigen::VectorXd(1);
    Eigen::VectorXd mGradient = Eigen::VectorXd(mMPCHorizon);
    Eigen::MatrixXd mNextStates = Eigen::MatrixXd(2,mMPCHorizon);
    Eigen::MatrixXd mNextStatesTemp = Eigen::MatrixXd(2,mMPCHorizon);
    Eigen::VectorXd mForce = Eigen::VectorXd(mMPCHorizon);
    Eigen::VectorXd mForceTemp = Eigen::VectorXd(mMPCHorizon);
    Eigen::VectorXd mNextX = Eigen::VectorXd(2);
    Eigen::VectorXd mNextXDes = Eigen::VectorXd(2);
};


#endif //RAISIM_SINGLELEGGEDMPCOPERATION_H
