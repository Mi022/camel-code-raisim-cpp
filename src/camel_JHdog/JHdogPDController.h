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

    Eigen::VectorXd torque = Eigen::VectorXd(3);
    raisim::VecDyn position = raisim::VecDyn(3);
    raisim::VecDyn velocity = raisim::VecDyn(3);
    double positionError[3];
    double velocityError[3];
    double desiredPosition[3];
    double desiredVelocity[3];

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
