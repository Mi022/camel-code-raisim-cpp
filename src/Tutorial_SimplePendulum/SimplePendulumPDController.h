//
// Created by jaehoon on 22. 4. 3..
//

#ifndef RAISIM_SIMPLEPENDULUMPDCONTROLLER_H
#define RAISIM_SIMPLEPENDULUMPDCONTROLLER_H

#include "include/CAMEL/Controller.h"
#include "include/TrajectoryGenerator/CubicTrajectoryGenerator.h"

class SimplePendulumPDController : public Controller {
public:
    Eigen::VectorXd torque = Eigen::VectorXd(1);
    double position;
    double velocity;
    double positionError;
    double velocityError;
    double desiredPosition;
    double desiredVelocity;

    double PGain;
    double DGain;

    SimplePendulumPDController(Robot *robot) : Controller(robot) {
        updateState();
        mTrajectoryGenerator.updateTrajectory(position, -90.0 / 180.0* 3.141592, getRobot() -> getWorldTime(), 5.0);
        setPDGain(200.0, 25.0);
    }

    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;
    void setPDGain(double PGain, double DGain);

private:
    CubicTrajectoryGenerator mTrajectoryGenerator;
};



#endif //RAISIM_SIMPLEPENDULUMPDCONTROLLER_H
