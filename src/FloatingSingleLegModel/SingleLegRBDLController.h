//
// Created by hwayoung on 22. 8. 17.
//

#ifndef RAISIM_SINGLELEGRBDLCONTROLLER_H
#define RAISIM_SINGLELEGRBDLCONTROLLER_H

#include "include/CAMEL/Controller.h"     //include Controller class, trajectory generator class
#include "include/TrajectoryGenerator/CubicTrajectoryGenerator.h"
#include "SingleLegRobot.h"

#include <rbdl/rbdl.h>

class SingleLegRBDLController : public Controller {
public:
    SingleLegRBDLController(SingleLegRobot *robot) : Controller(robot) {
        mCubicTrajectoryGeneratorNd = CubicTrajectoryGeneratorND(robot->dim - 4);
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
    CubicTrajectoryGeneratorND mCubicTrajectoryGeneratorNd = CubicTrajectoryGeneratorND(5);
    bool mIsGenerateTrajectory;
    bool mIsTrajectoryAlready;
    double mTimeDuration;
};


#endif //RAISIM_SINGLELEGRBDLCONTROLLER_H
