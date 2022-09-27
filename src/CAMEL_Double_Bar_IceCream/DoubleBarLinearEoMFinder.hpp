//
// Created by hwayoung on 22. 9. 26.
//

#ifndef RAISIM_DOUBLEBARLINEAREOMFINDER_HPP
#define RAISIM_DOUBLEBARLINEAREOMFINDER_HPP

#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "eFactor.hpp"

class DoubleBarLinearEoMFinder
{
public:
    DoubleBarLinearEoMFinder(std::string urdfPath, Eigen::VectorXd desiredPosition, Eigen::VectorXd desiredVelocity, Eigen::VectorXd desiredTorque, double dT);

    const Eigen::MatrixXd& GetA() const;
    const Eigen::MatrixXd& GetB() const;

    void FindAB();

private:
    void getModelFromURDF(std::string urdfPath);
    Eigen::MatrixXd computePartialDiff(eFactor factor);
    void generateAc();
    void generateBc();
    void generateABd();
    void testAB();

    const double mDT;
    Eigen::MatrixXd mA;
    Eigen::MatrixXd mB;
    const Eigen::VectorXd mDesiredPosition;
    const Eigen::VectorXd mDesiredVelocity;
    const Eigen::VectorXd mDesiredTorque;
    Eigen::VectorXd mQDDot;
    RigidBodyDynamics::Model* mModel;

    static const double mDelta;
};


#endif //RAISIM_DOUBLEBARLINEAREOMFINDER_HPP
