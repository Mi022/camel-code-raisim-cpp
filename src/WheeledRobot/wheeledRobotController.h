//
// Created by ljm on 22. 7. 6.
//

#ifndef RAISIM_WHEELEDROBOTCONTROLLER_H
#define RAISIM_WHEELEDROBOTCONTROLLER_H

#include "include/CAMEL/Controller.h"
#include "wheeledRobot.h"

class wheeledRobotController : public Controller {
public:
    Eigen::VectorXd torque = Eigen::VectorXd(10);
    raisim::VecDyn position = raisim::VecDyn(10);
    raisim::VecDyn velocity = raisim::VecDyn(10);
    raisim::VecDyn lastVelocity = raisim::VecDyn(10);
    Eigen::VectorXd velocityError = Eigen::VectorXd(4);
    raisim::VecDyn desiredVelocity = raisim::VecDyn(4);

    double PGain;
    double torqueLimit = 30.0;

    int vel;
    int velLimit = 30;

    wheeledRobotController(Robot *robot) : Controller(robot) {
        torque.setZero();
        desiredVelocity.setZero();

        setPGain(100.0);
    }

    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;
    void setForward();
    void setLeft();
    void setRight();
    void setBack();
    void setStop();
    void accelerate();
    void setVel(int val);
    void reset();

    void setPGain(double PGain);

private:

};

#endif //RAISIM_WHEELEDROBOTCONTROLLER_H
