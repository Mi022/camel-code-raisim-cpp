//
// Created by hwayoung on 22. 9. 22.
//

#ifndef RAISIM_DOUBLEBARRBDLCONTROLLER_H
#define RAISIM_DOUBLEBARRBDLCONTROLLER_H

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "include/CAMEL/Controller.h"
#include "DoubleBarRobot.h"
#include "DoubleBarSteadyStateCalculator.h"

class DoubleBarRBDLController : public Controller {
public:
    DoubleBarRBDLController(DoubleBarRobot *robot) : Controller(robot) {
        getModelFromURDF();
        torque = Eigen::VectorXd(robot->dim);
        position = getRobot() -> getQ().e();
        velocity = getRobot() -> getQD().e();
        DoubleBarSteadyStateCalculator steadyStateCalculator = DoubleBarSteadyStateCalculator(model, position, velocity);
        std::cout<<"optimal tau: "<<steadyStateCalculator.tau<<std::endl;

        desiredPosition = Eigen::VectorXd(robot->dim).setZero();
        desiredVelocity = Eigen::VectorXd(robot->dim).setZero();
        desiredAccerelation = Eigen::VectorXd(robot->dim).setZero();
    }

    Eigen::VectorXd torque;
    Eigen::VectorXd position;
    Eigen::VectorXd velocity;
    Eigen::VectorXd desiredPosition;
    Eigen::VectorXd desiredVelocity;
    Eigen::VectorXd desiredAccerelation;

    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;

private:
    void getModelFromURDF();
    RigidBodyDynamics::Model* model;

};


#endif //RAISIM_DOUBLEBARRBDLCONTROLLER_H
