//
// Created by jaehyeong on 22. 8. 16.
//

#ifndef RAISIM_JHDOGPDCONTROLLER_H
#define RAISIM_JHDOGPDCONTROLLER_H

#include "include/CAMEL/Controller.h"
#include "include/TrajectoryGenerator/CubicTrajectoryGenerator.h"

//extend Controller class (parent) to JHlegPDcontroller class (child)
class JHdogPDController : public Controller {
public:

    JHdogPDController(Robot *robot) : Controller(robot) {
        updateState();
        setPDGain(200,25);
    }

    Eigen::VectorXd torque = Eigen::VectorXd(4);
    raisim::VecDyn position = raisim::VecDyn(4);
    raisim::VecDyn velocity = raisim::VecDyn(4);
    double positionError;
    double velocityError;
    double desiredPosition;
    double desiredVelocity;

    double PGain;
    double DGain;

    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;
    void setPDGain(double PGain, double DGain);

private:
    CubicTrajectoryGenerator mTrajectoryGenerator;
};
#endif //RAISIM_JHDOGPDCONTROLLER_H
