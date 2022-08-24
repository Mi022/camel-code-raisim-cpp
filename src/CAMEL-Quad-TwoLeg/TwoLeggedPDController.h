
#ifndef RAISIM_TWOLEGGEDPDCONTROLLER_H
#define RAISIM_TWOLEGGEDPDCONTROLLER_H

#include "include/CAMEL/Controller.h"

class TwoLeggedPDController : public Controller {
public:
    Eigen::VectorXd torque = Eigen::VectorXd(10);
    raisim::VecDyn position = raisim::VecDyn(11);
    raisim::VecDyn velocity = raisim::VecDyn(10);
    raisim::VecDyn desiredPosition = raisim::VecDyn(11);
    raisim::VecDyn desiredVelocity = raisim::VecDyn(10);
    Eigen::VectorXd positionError = Eigen::VectorXd(4);
    Eigen::VectorXd velocityError = Eigen::VectorXd(4);
    Eigen::VectorXd PGain = Eigen::VectorXd(4);
    Eigen::VectorXd DGain = Eigen::VectorXd(4);

    double torqueLimit = 13.0;

    TwoLeggedPDController(Robot *robot) : Controller(robot) {
//        mTrajectoryGenerator.updateTrajectory(position[0], 0.35, getRobot()->getWorldTime(), 1.0);
        setPDGain({80.0, 100.0, 80.0, 100.0}, {10.0, 4.0, 10.0, 4.0});
//        setPDGain({0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0});
        mSetDesired();
        torque.setZero();
    }

    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;
    void setPDGain(Eigen::Vector4d PGain, Eigen::Vector4d DGain);
    void IKsolve();

private:
    void mSetDesired();
    static const double mass;
    static const double gravity;
    static const double l;
    static const double deg2rad;
    Eigen::VectorXd mTheta = Eigen::VectorXd(4);//RH = 0, RK = 1, LH = 2, LK = 3;
};


#endif //RAISIM_TWOLEGGEDPDCONTROLLER_H
