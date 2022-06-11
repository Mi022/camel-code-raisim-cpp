//
// Created by user on 22. 6. 10.
//

#ifndef RAISIM_MIPPDCONTROLLER_H
#define RAISIM_MIPPDCONTROLLER_H

#include "include/CAMEL/Controller.h"
//#include <random>

class MIPPDController : public Controller{
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
    double desiredPosition;
    double desiredVelocity;

    double PGain;
    double DGain;
    double torqueLimit = 3.5;

    MIPPDController(Robot *robot) : Controller(robot) {
        setTrajectory();
        setPDGain(7.0, 0.4);
        torque.setZero();
        externalForce.setZero();
        forcePosition.setZero();

        forcePosition[0] = 0.0;
        forcePosition[1] = 0.0;
        forcePosition[2] = 0.08;

        externalForce[0] = 0.0;
        externalForce[1] = 4.0;
        externalForce[2] = 0.0;

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
};


#endif //RAISIM_MIPPDCONTROLLER_H
