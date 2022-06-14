//
// Created by user on 22. 6. 14.
//

#ifndef RAISIM_MIPPDDCONTROLLER_H
#define RAISIM_MIPPDDCONTROLLER_H

#include "include/CAMEL/Controller.h"

class MIPPDDController : public Controller{
public:
    Eigen::VectorXd torque = Eigen::VectorXd(2);
    raisim::VecDyn position = raisim::VecDyn(2);
    raisim::VecDyn velocity = raisim::VecDyn(2);
    Eigen::VectorXd positionError = Eigen::VectorXd(1);
    Eigen::VectorXd velocityError = Eigen::VectorXd(1);
    Eigen::VectorXd desiredJointPosition = Eigen::VectorXd(1);
    Eigen::VectorXd desiredJointVelocity = Eigen::VectorXd(1);

    raisim::Vec<3> externalForce;
    raisim::Vec<3> forcePosition;
    double motorVelocityError;
    double desiredMotorVelocity;

    double PGain;
    double DGain;
    double torqueLimit = 3.5;

    MIPPDDController(Robot *robot) : Controller(robot) {
        setTrajectory();
        setPDGain(6.08431, 0.607407);
        torque.setZero();
        i = 0;
    }

    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;
    void setPDGain(double PGain, double DGain);

private:
    int i;
    void generateExternalForce();
    void addNoise();
};




#endif //RAISIM_MIPPDDCONTROLLER_H
