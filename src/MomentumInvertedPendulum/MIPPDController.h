//
// Created by user on 22. 6. 10.
//

#ifndef RAISIM_MIPPDCONTROLLER_H
#define RAISIM_MIPPDCONTROLLER_H

#include "include/CAMEL/Controller.h"

#define FORCE_DURATION 400
#define MAX_POSITION_NOISE 0.001 //withstand 0.05
#define MAX_VELOCITY_NOISE 0.01 //withstand 0.6
#define RANDOM_BOUNDARY 100

class MIPPDController : public Controller{
public:

    MIPPDController(Robot *robot) : Controller(robot) {
        setTrajectory();
        setPDGain(7.0, 0.4);
        mTorque.setZero();
        mIteration = 0;
    }

    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;
    void setPDGain(double PGain, double DGain);
    const Eigen::VectorXd &getTorque() const;

private:
    raisim::VecDyn mPosition = raisim::VecDyn(2);
    raisim::VecDyn mVelocity = raisim::VecDyn(2);
    Eigen::VectorXd mTorque = Eigen::VectorXd(2);
    Eigen::VectorXd mDesiredJointPosition = Eigen::VectorXd(1);
    Eigen::VectorXd mDesiredJointVelocity = Eigen::VectorXd(1);
    raisim::Vec<3> mExternalForce;
    raisim::Vec<3> mForcePosition;
    double mDesiredRodPosition;
    double mDesiredRodVelocity;
    double mPositionError;
    double mVelocityError;
    double mPGain;
    double mDGain;
    double mTorqueLimit = 3.5;
    int mIteration;

    void generateExternalForce();
    void addNoise();
};


#endif //RAISIM_MIPPDCONTROLLER_H
