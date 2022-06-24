//
// Created by hs on 22. 6. 20.
//

#ifndef RAISIM_SINGLELEGGEDMPCQPOASES_H
#define RAISIM_SINGLELEGGEDMPCQPOASES_H

#include "include/CAMEL/Controller.h"
//#include "include/TrajectoryGenerator/QuinticTrajectoryGenerator.h"
#include "include/TrajectoryGenerator/SincurveTrajectoryGenerator.h"
#include <qpOASES.hpp>
#include <unsupported/Eigen/MatrixFunctions>

using Eigen::Dynamic;

class SingleLeggedMPCqpoases : public Controller {
public:
    raisim::VecDyn position = raisim::VecDyn(3);
    raisim::VecDyn velocity = raisim::VecDyn(3);

    Eigen::VectorXd torque = Eigen::VectorXd(3);

    double desiredPosition;
    double desiredVelocity;

    double dz_dth1 = 0.0;
    double dz_dth2 = 0.0;
    double calculatedForce = 0.0;
    double torqueLimit = 13.0;

    SingleLeggedMPCqpoases(Robot *robot, double dT) : Controller(robot) {
        Aqp.setZero();
        Bqp.setZero();
        xd.setZero();

        updateState();
        mTrajectoryGenerator.updateTrajectory(position[0], getRobot()->getWorldTime(), 1.0);
        mDT = dT;
    }
    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;
    void getMatrices();
    void qpSolver();
    void matrix_to_real(qpOASES::real_t* dst, Eigen::Matrix<double,Dynamic,Dynamic> src, int16_t rows, int16_t cols);

private:
    //QuinticTrajectoryGenerator mTrajectoryGenerator;
    SincurveTrajectoryGenerator mTrajectoryGenerator;

    double mLumpedMass = 2.009;
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
