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

    wheeledRobotController(Robot *robot) : Controller(robot) {
        torque.setZero();
    }

    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;

private:

};

#endif //RAISIM_WHEELEDROBOTCONTROLLER_H
