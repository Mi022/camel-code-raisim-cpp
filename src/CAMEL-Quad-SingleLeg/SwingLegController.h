//
// Created by hs on 22. 9. 16.
//

#ifndef RAISIM_SWINGLEGCONTROLLER_H
#define RAISIM_SWINGLEGCONTROLLER_H

#include "include/CAMEL/Controller.h"
#include "include/TrajectoryGenerator/legtrajectory.h"


class SwingLegController : public Controller {
public:
    Eigen::VectorXd torque = Eigen::VectorXd(2);
    raisim::VecDyn position = raisim::VecDyn(2);
    raisim::VecDyn velocity = raisim::VecDyn(2);
    Eigen::VectorXd positionError = Eigen::VectorXd(2);
    Eigen::VectorXd velocityError = Eigen::VectorXd(2);
    Eigen::VectorXd desiredJointPosition = Eigen::VectorXd(2);
    Eigen::VectorXd desiredJointVelocity = Eigen::VectorXd(2);

    double desiredPosition[2];
    double desiredVelocity[2];
    raisim::Vec<3> footPosition;

    double PGain[2] = {30 , 30};
    double DGain[2] = {1 , 1};
    double torqueLimit = 13.0;

    SwingLegController(Robot *robot) : Controller(robot) {
            mTrajectoryGenerator.updateTrajectory(getRobot()->getWorldTime(), 1);
    }

    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;
    void IKsolve();

private:
    legtrajectory mTrajectoryGenerator;
};

#endif //RAISIM_SWINGLEGCONTROLLER_H
