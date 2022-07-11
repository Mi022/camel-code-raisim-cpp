//
// Created by hs on 22. 6. 27.
//

#ifndef RAISIM_A1MPCCONTROLLER_H
#define RAISIM_A1MPCCONTROLLER_H

#include "include/CAMEL/Controller.h"
#include "A1Robot.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <qpOASES.hpp>

using Eigen::Dynamic;

class A1MPCController : public Controller {
public:
    raisim::VecDyn position = raisim::VecDyn(19);
    raisim::VecDyn velocity = raisim::VecDyn(18);
    raisim::VecDyn torque = raisim::VecDyn(18);

    double l1 = 0.085;
    double l2 = 0.2;
    double l3 = 0.2;

    double torqueLimit = 20.0;
    Eigen::Matrix<double,3,1> f[4];
    Eigen::Matrix<double,3,3> robotJacobian[4];
    Eigen::Matrix<double,3,1> robottorque[4];

    Eigen::Matrix<double, 4, 1> quat;
    Eigen::Matrix<double, 3, 1> p;
    Eigen::Matrix<double, 3, 1> v;
    Eigen::Matrix<double, 3, 1> w;
    Eigen::Matrix<double, 3, 1> q;

    double desiredPositionX = 0.f;
    double desiredPositionY = 0.f;
    double desiredPositionZ = 0.f;

    double desiredRotationX = 0.f;
    double desiredRotationY = 0.f;
    double desiredRotationZ = 0.f;

    A1MPCController(Robot *robot, double dT) : Controller(robot){
        mDT = dT;
        K.setIdentity();

        Aqp.setZero();
        Bqp.setZero();
        L.setZero();
        H.setZero();
        g.setZero();
        U_b.setZero();
        fmat.setZero();

        torque.setZero();

    }
    void doControl() override;
    void updateState() override;
    void setTrajectory() override;
    void getMetrices();
    void qpSolver();
    void computeControlInput() override;
    void setControlInput() override;

    void quat_to_euler(Eigen::Matrix<double,4,1>& quat, Eigen::Matrix<double,3,1>& q);
    void ss_mats(Eigen::Matrix<double,13,13>& Ac, Eigen::Matrix<double,13,12>& Bc);
    void c2qp(Eigen::Matrix<double,13,13> A, Eigen::Matrix<double,13,12> B);
    void matrix_to_real(qpOASES::real_t* dst, Eigen::Matrix<double,Dynamic,Dynamic> src, int16_t rows, int16_t cols);
    void getJacobian(Eigen::Matrix<double,3,3>& J, double hip, double thigh, double calf, int side);

private:
    double mLumpedMass = 30;
    double mGravity = -9.81;
    int mMPCHorizon = 5;
    double mDT;

    double alpha = 1e-10;

    Eigen::MatrixXd x0 = Eigen::MatrixXd(13, 1);
    Eigen::MatrixXd xd = Eigen::MatrixXd(13*mMPCHorizon, 1);

    Eigen::MatrixXd Aqp = Eigen::MatrixXd(13*mMPCHorizon, 13);
    Eigen::MatrixXd Bqp = Eigen::MatrixXd(13*mMPCHorizon, 12*mMPCHorizon);

    Eigen::MatrixXd L = Eigen::MatrixXd(13*mMPCHorizon, 13*mMPCHorizon);
    Eigen::MatrixXd K = Eigen::MatrixXd(12*mMPCHorizon, 12*mMPCHorizon);
    Eigen::MatrixXd H = Eigen::MatrixXd(12*mMPCHorizon, 12*mMPCHorizon);
    Eigen::MatrixXd g = Eigen::MatrixXd(12*mMPCHorizon, 1);

    Eigen::MatrixXd U_b = Eigen::MatrixXd(20*mMPCHorizon,1);
    Eigen::MatrixXd fmat = Eigen::MatrixXd(20*mMPCHorizon,12*mMPCHorizon);
};

#endif //RAISIM_A1MPCCONTROLLER_H
