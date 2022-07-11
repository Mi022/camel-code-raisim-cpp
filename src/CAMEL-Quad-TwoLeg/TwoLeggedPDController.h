//
// Created by jaehoon on 22. 4. 3..
//

#ifndef RAISIM_SIMPLEPENDULUMPDCONTROLLER_H
#define RAISIM_SIMPLEPENDULUMPDCONTROLLER_H

#include "include/CAMEL/Controller.h"
#include "include/TrajectoryGenerator/QuinticTrajectoryGenerator.h"

class SingleLeggedPDController : public Controller {
public:
    Eigen::VectorXd torque = Eigen::VectorXd(3);
    raisim::VecDyn position = raisim::VecDyn(3);
    raisim::VecDyn velocity = raisim::VecDyn(3);
    Eigen::VectorXd positionError = Eigen::VectorXd(2);
    Eigen::VectorXd velocityError = Eigen::VectorXd(2);
    Eigen::VectorXd desiredJointPosition = Eigen::VectorXd(2);
    Eigen::VectorXd desiredJointVelocity = Eigen::VectorXd(2);

    double desiredPosition;
    double desiredVelocity;

    double PGain;
    double DGain;
    double torqueLimit = 13.0;

    SingleLeggedPDController(Robot *robot) : Controller(robot) {
        mTrajectoryGenerator.updateTrajectory(position[0], 0.35, getRobot()->getWorldTime(), 1.0);
        setPDGain(50.0, 1.5);
        torque[0] = 0.0;
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
};


#endif //RAISIM_SIMPLEPENDULUMPDCONTROLLER_H
