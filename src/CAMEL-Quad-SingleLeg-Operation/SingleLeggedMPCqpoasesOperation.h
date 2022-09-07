//
// Created by hs on 22. 6. 20.
//

#ifndef RAISIM_SINGLELEGGEDMPCQPOASES_H
#define RAISIM_SINGLELEGGEDMPCQPOASES_H

#include "SingleLeggedRobotOperation.h"
#include "include/TrajectoryGenerator/QuinticTrajectoryGenerator.h"
//#include "include/TrajectoryGenerator/SincurveTrajectoryGenerator.h"
#include <qpOASES.hpp>
#include <unsupported/Eigen/MatrixFunctions>

using Eigen::Dynamic;

class SingleLeggedMPCqpoases {
public:
    raisim::VecDyn position = raisim::VecDyn(3);
    raisim::VecDyn velocity = raisim::VecDyn(3);
    Eigen::VectorXd torque = Eigen::VectorXd(2);

    double desiredPosition;
    double desiredVelocity;

    double dz_dth1 = 0.0;
    double dz_dth2 = 0.0;
    double calculatedForce = 0.0;

    SingleLeggedMPCqpoases(SingleLeggedRobotOperation *robot, double *currentTime, double dT) {
        mRobot = robot;
        mCurrentTime = currentTime;
        Aqp.setZero();
        Bqp.setZero();
        xd.setZero();
        torque.setZero();
        mDT = dT;
    }
    void doControl();
    void setTrajectory();
    void updateState();
    void computeControlInput();
    void setControlInput();
    void getMatrices();
    void qpSolver();
    void matrix_to_real(qpOASES::real_t* dst, Eigen::Matrix<double,Dynamic,Dynamic> src, int16_t rows, int16_t cols);

    void setPointTrajectory(double goalPosition);
    void zeroing();
    void updateCubicTrajectory(double goalPosition, double timeDuration);
//    void updateSinTrajectory(double amplitude, double frequency, double timeDuration);
    void setPointTrajectoryZeroing();
private:
    SingleLeggedRobotOperation *mRobot;
    QuinticTrajectoryGenerator mTrajectoryGenerator;
//    SincurveTrajectoryGenerator mTrajectoryGenerator;
    bool mIsZeroing = false;
    bool mIsCubic = false;
    bool mIsSin = false;
    double mTorqueLimit = 10.0;
    double mHaltTime = 0.0;
    double *mCurrentTime;
    double mLumpedMass = 2.766;
    double mGravity = -9.81;
    double mDT;

    int mMPCHorizon = 5;
    double mInitialForce = 19.7083;
    Eigen::MatrixXd x0 = Eigen::MatrixXd(3, 1);
    Eigen::MatrixXd xd = Eigen::MatrixXd(3*mMPCHorizon, 1);
    Eigen::VectorXd mForce = Eigen::VectorXd(mMPCHorizon);

    Eigen::Matrix<double,3,3> A;
    Eigen::Matrix<double,3,1> B;
    Eigen::MatrixXd Aqp = Eigen::MatrixXd(3*mMPCHorizon, 3);
    Eigen::MatrixXd Bqp = Eigen::MatrixXd(3*mMPCHorizon, mMPCHorizon);
    Eigen::MatrixXd L = Eigen::MatrixXd(3*mMPCHorizon, 3*mMPCHorizon);
    Eigen::MatrixXd K = Eigen::MatrixXd(mMPCHorizon, mMPCHorizon);
    Eigen::MatrixXd H = Eigen::MatrixXd(mMPCHorizon, mMPCHorizon);
    Eigen::MatrixXd g = Eigen::MatrixXd(mMPCHorizon, 1);

};

#endif //RAISIM_SINGLELEGGEDMPCQPOASES_H
