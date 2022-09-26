//
// Created by hwayoung on 22. 9. 22.
//

#ifndef RAISIM_DOUBLEBARSTEADYSTATECALCULATOR_H
#define RAISIM_DOUBLEBARSTEADYSTATECALCULATOR_H

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

class DoubleBarSteadyStateCalculator {
public:
    DoubleBarSteadyStateCalculator(RigidBodyDynamics::Model* model, Eigen::VectorXd desiredPosition, Eigen::VectorXd desiredVelocity)
    : mModel(model) ,mDesiredPosition(desiredPosition), mDesiredVelocity(desiredVelocity)
    {
        mQDDot = Eigen::VectorXd::Zero (mModel->qdot_size);
        mTau = Eigen::VectorXd::Zero (model->qdot_size);

        mIteration = 0;
        mDelta = 1e-3;
        mStepSize = 5*1e-5;
        mTerminateCondition = 1e-6;
        mMaximumIteration = 10000;
        mRMSGradient = 10000;
        RigidBodyDynamics::InverseDynamics(*model, mDesiredPosition, mDesiredVelocity, mQDDot, mTau);

//        solveTorque();
    }
    void SolveTorque();

    const Eigen::VectorXd &getTau() const;

private:
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
