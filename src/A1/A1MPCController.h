//
// Created by hs on 22. 6. 27.
//

#ifndef RAISIM_A1MPCCONTROLLER_H
#define RAISIM_A1MPCCONTROLLER_H

#include "include/CAMEL/Controller.h"
#include "A1Robot.h"

class A1MPCController : public Controller {
public:
    raisim::VecDyn position = raisim::VecDyn(19);
    raisim::VecDyn velocity = raisim::VecDyn(18);

    Eigen::VectorXd torque = Eigen::VectorXd(18);

    double torqueLimit = 20.0;

    A1MPCController(Robot *robot, double dT) : Controller(robot){
        updateState();
        mDT = dT;
    }
    void doControl() override;
    void updateState() override;
    void setTrajectory() override;
    void getMetrices();
    void computeControlInput() override;
    void setControlInput() override;

    void quat_to_euler(Eigen::Matrix<double,4,1>& quat, Eigen::Matrix<double,3,1>& q);
    void ss_mats(Eigen::Matrix<double,13,13>& Ac, Eigen::Matrix<double,13,12>& Bc);
    void c2qp(Eigen::Matrix<double,13,13> A, Eigen::Matrix<double,13,12> B);

private:
    double mLumpedMass = 13.f;
    double mGravity = -9.81;
    int mMPCHorizon = 1;
    double mDT;

    Eigen::Matrix<double, 4, 1> quat;
    Eigen::Matrix<double, 3, 1> p;
    Eigen::Matrix<double, 3, 1> q;
    Eigen::Matrix<double, 3, 1> v;
    Eigen::Matrix<double, 3, 1> w;

    Eigen::MatrixXd x0 = Eigen::MatrixXd(13, 1);
    Eigen::MatrixXd xd = Eigen::MatrixXd(13*mMPCHorizon, 1);

    Eigen::MatrixXd Aqp = Eigen::MatrixXd(13*mMPCHorizon, 13);
    Eigen::MatrixXd Bqp = Eigen::MatrixXd(13*mMPCHorizon, 12*mMPCHorizon);
};

#endif //RAISIM_A1MPCCONTROLLER_H
