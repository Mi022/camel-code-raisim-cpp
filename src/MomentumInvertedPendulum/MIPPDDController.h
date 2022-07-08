//
// Created by user on 22. 6. 14.
//

#ifndef RAISIM_MIPPDDCONTROLLER_H
#define RAISIM_MIPPDDCONTROLLER_H

#include "include/CAMEL/Controller.h"

class MIPPDDController : public Controller{
public:

    MIPPDDController(Robot *robot) : Controller(robot) {
        setTrajectory();
        setPDDGain(6.08431, 0.607407, 0.0254538);
        setTorqueLimit(3.5);
        mTorque.setZero();
        i = 0;
    }

    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;
    void setPDDGain(double PGain, double DGain, double DDGain);
    void setTorqueLimit(double torqueLimit){this->mTorqueLimit = torqueLimit;}
    const Eigen::VectorXd &getTorque() const;
    const Eigen::VectorXd &getDesiredJointPosition() const;
    const Eigen::VectorXd &getDesiredJointVelocity() const;
    double getDesiredMotorVelocity() const;

private:
    Eigen::VectorXd mTorque = Eigen::VectorXd(2);
    raisim::VecDyn mPosition = raisim::VecDyn(2);
    raisim::VecDyn mVelocity = raisim::VecDyn(2);
    Eigen::VectorXd mPositionError = Eigen::VectorXd(1);
    Eigen::VectorXd mVelocityError = Eigen::VectorXd(1);
    Eigen::VectorXd mDesiredJointPosition = Eigen::VectorXd(1);
    Eigen::VectorXd mDesiredJointVelocity = Eigen::VectorXd(1);

    raisim::Vec<3> mExternalForce;
    raisim::Vec<3> mForcePosition;
    double mMotorVelocityError;
    double mDesiredMotorVelocity;

    double mPGain;
    double mDGain;
    double mDDGain;
    double mTorqueLimit;

    int i;
    void generateExternalForce();
    void addNoise();
};




#endif //RAISIM_MIPPDDCONTROLLER_H
