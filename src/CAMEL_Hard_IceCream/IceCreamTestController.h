//
// Created by hwayoung on 22. 8. 29.
//

#ifndef RAISIM_ICECREAMTESTCONTROLLER_H
#define RAISIM_ICECREAMTESTCONTROLLER_H

#include "include/CAMEL/Controller.h"     //include Controller class, trajectory generator class
#include "include/TrajectoryGenerator/CubicTrajectoryGenerator.h"
#include "IceCreamRobot.h"

class IceCreamTestController : public Controller {
public:
    IceCreamTestController(IceCreamRobot *robot) : Controller(robot) {
        torque = Eigen::VectorXd(robot->dim - 1);
        position = getRobot() -> getQ().e();
        velocity = getRobot() -> getQD().e();

        desiredPosition = getRobot() -> getQ().e();
        desiredVelocity = getRobot() -> getQD().e();

        mTimeDuration = 1.0;
        mIsGenerateTrajectory = false;
        mIsTrajectoryAlready = false;

    }

    Eigen::VectorXd torque;
    Eigen::VectorXd position;
    Eigen::VectorXd velocity;
    Eigen::VectorXd desiredPosition;
    Eigen::VectorXd desiredVelocity;

    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;

private:
    bool mIsGenerateTrajectory;
    bool mIsTrajectoryAlready;
    double mTimeDuration;
};



#endif //RAISIM_ICECREAMTESTCONTROLLER_H
