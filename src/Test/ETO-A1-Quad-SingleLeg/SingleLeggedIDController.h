//
// Created by jaehoon on 22. 6. 6.
//

#ifndef RAISIM_SINGLELEGGEDIDCONTROLLER_H
#define RAISIM_SINGLELEGGEDIDCONTROLLER_H

#include "include/CAMEL/Controller.h"
#include "include/TrajectoryGenerator/QuinticTrajectoryGenerator.h"

class SingleLeggedIDController : public Controller {
public:
    Eigen::VectorXd torque = Eigen::VectorXd(3);

    raisim::VecDyn position = raisim::VecDyn(3);
    raisim::VecDyn velocity = raisim::VecDyn(3);
    Eigen::VectorXd positionError = Eigen::VectorXd(2);
    Eigen::VectorXd velocityError = Eigen::VectorXd(2);
    Eigen::VectorXd desiredJointPosition = Eigen::VectorXd(2);
    Eigen::VectorXd desiredJointPosition_past = Eigen::VectorXd(2);
    Eigen::VectorXd desiredJointVelocity = Eigen::VectorXd(2);

    double desiredPosition;
    double desiredVelocity;
    double desiredAcceleration;
    double calculatedForce = 0.0;

    // jacobian
    double dz_dth1 = 0.0;
    double dz_dth2 = 0.0;

    double PGain;
    double DGain;
    double torqueLimit = 10.0;



    SingleLeggedIDController(Robot *robot, double dT) : Controller(robot) {
        updateState();
        mTrajectoryGenerator.updateTrajectory(position[0], 0.35, getRobot()->getWorldTime(), 1.0);
        setPDGain(50.0, 2.5);
        torque[0] = 0.0;
        mDT = dT;
    }

    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;
    void setPDGain(double PGain, double DGain);
    void IKsolve();

private:
    QuinticTrajectoryGenerator mTrajectoryGenerator;
    double mLumpedMass = 2.009;
    double mGravity = -9.81;
    double mDT;
};

#endif //RAISIM_SINGLELEGGEDIDCONTROLLER_H
