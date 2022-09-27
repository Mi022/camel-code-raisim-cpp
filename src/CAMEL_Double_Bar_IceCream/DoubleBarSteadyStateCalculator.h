//
// Created by hwayoung on 22. 9. 22.
//

#ifndef RAISIM_DOUBLEBARSTEADYSTATECALCULATOR_H
#define RAISIM_DOUBLEBARSTEADYSTATECALCULATOR_H

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

class DoubleBarSteadyStateCalculator
{
public:
    DoubleBarSteadyStateCalculator(std::string urdfPath, Eigen::VectorXd desiredPosition, Eigen::VectorXd desiredVelocity)
            : mDesiredPosition(desiredPosition)
            , mDesiredVelocity(desiredVelocity)
    {
        getModelFromURDF(urdfPath);

        mQDDot = Eigen::VectorXd::Zero(mModel->qdot_size);
        mTau = Eigen::VectorXd::Zero(mModel->qdot_size);

        mIteration = 0;
        mDelta = 1e-3;
        mStepSize = 5 * 1e-5;
        mTerminateCondition = 1e-6;
        mMaximumIteration = 10000;
        mRMSGradient = 10000;
        RigidBodyDynamics::InverseDynamics(*mModel, mDesiredPosition, mDesiredVelocity, mQDDot, mTau);
    }

    void SolveTorque();

    const Eigen::VectorXd& getTau() const;

private:
    void getModelFromURDF(std::string urdfPath);
    double h(Eigen::VectorXd tau);
    double gradientH(int tauIndex);
    void updateTau();

    RigidBodyDynamics::Model* mModel;
    Eigen::VectorXd mDesiredPosition;
    Eigen::VectorXd mDesiredVelocity;
    Eigen::VectorXd mQDDot;
    Eigen::VectorXd mTau;
    double mDelta;
    double mStepSize;
    double mTerminateCondition;
    double mRMSGradient;
    int mMaximumIteration;
    int mIteration;
};


#endif //RAISIM_DOUBLEBARSTEADYSTATECALCULATOR_H
