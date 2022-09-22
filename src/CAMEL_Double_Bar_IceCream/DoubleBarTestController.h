//
// Created by hwayoung on 22. 8. 29.
//

#ifndef RAISIM_DOUBLEBARTESTCONTROLLER_H
#define RAISIM_DOUBLEBARTESTCONTROLLER_H

#include "include/CAMEL/Controller.h"     //include Controller class, trajectory generator class
#include "include/TrajectoryGenerator/CubicTrajectoryGenerator.h"
#include "DoubleBarRobot.h"

class DoubleBarTestController : public Controller {
public:
    DoubleBarTestController(DoubleBarRobot *robot) : Controller(robot) {
        torque = Eigen::VectorXd(robot->dim);
        position = getRobot() -> getQ().e();
        velocity = getRobot() -> getQD().e();

        desiredPosition = getRobot() -> getQ().e();
        desiredVelocity = getRobot() -> getQD().e();

        PastVelocity = getRobot() -> getQD().e();

        mTrajectoryGenerator = CubicTrajectoryGenerator();
        mTrajectoryGenerator.updateTrajectory(position[1], -90*robot->deg2rad, robot->getWorldTime(), 3.0);

        mPGain = 1.0;
        mDGain = 0.4;
    }

    Eigen::VectorXd torque;
    Eigen::VectorXd position;
    Eigen::VectorXd velocity;
    Eigen::VectorXd desiredPosition;
    Eigen::VectorXd desiredVelocity;
    Eigen::VectorXd PastVelocity;

    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;

private:
    double mPGain, mDGain;
    CubicTrajectoryGenerator mTrajectoryGenerator;
};



#endif //RAISIM_DOUBLEBARTESTCONTROLLER_H
