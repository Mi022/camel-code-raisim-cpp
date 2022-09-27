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
#include "DoubleBarStablePointFinder.h"
#include "DoubleBarSteadyStateCalculator.h"

class DoubleBarRBDLController : public Controller
{
public:
    DoubleBarRBDLController(DoubleBarRobot* robot, std::string urdfPath)
        : Controller(robot)
    {
        getModelFromURDF();
        torque = Eigen::VectorXd(robot->getDim());
        position = getRobot()->getQ().e();
        velocity = getRobot()->getQD().e();
        DoubleBarStablePointFinder stablePointFinder = DoubleBarStablePointFinder(position, robot);
        stablePointFinder.FindStableState();
        std::cout << "optimal stable point: " << std::endl << stablePointFinder.getPosition() << std::endl;
        position = stablePointFinder.getPosition();
        desiredPosition = stablePointFinder.getPosition();
        DoubleBarSteadyStateCalculator steadyStateCalculator = DoubleBarSteadyStateCalculator(urdfPath, position, velocity);
        steadyStateCalculator.SolveTorque();
        std::cout << "optimal tau: " << steadyStateCalculator.getTau() << std::endl;
        torque = steadyStateCalculator.getTau();
        std::cout << "COM: " << robot->robot->getCOM() << std::endl;

        desiredPosition = Eigen::VectorXd(robot->getDim()).setZero();
        desiredVelocity = Eigen::VectorXd(robot->getDim()).setZero();
        desiredAccerelation = Eigen::VectorXd(robot->getDim()).setZero();
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
