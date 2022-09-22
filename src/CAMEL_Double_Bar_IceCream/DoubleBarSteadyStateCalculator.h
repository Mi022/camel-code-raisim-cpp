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
    DoubleBarSteadyStateCalculator(RigidBodyDynamics::Model* model, Eigen::VectorXd desiredPosition, Eigen::VectorXd desiredVelocity){
        mModel = model;
        mDesiredPosition = desiredPosition;
        mDesiredVelocity = desiredVelocity;

        mIteration = 0;
        mDelta = 1e-3;
        mStepSize = 5*1e-6;
        mTerminateCondition = 1e-6;
        mMaximumIteration = 10000;
        mRMSGradient = 10000;
        tau = Eigen::VectorXd::Zero (model->qdot_size);
        tau << 0.0, 1.10556, -3.86885e-06;


        solve();
    }
    Eigen::VectorXd tau;

private:
    RigidBodyDynamics::Model* mModel;
    Eigen::VectorXd mDesiredPosition;
    Eigen::VectorXd mDesiredVelocity;
    double mDelta;
    double mStepSize;
    double mTerminateCondition;
    double mRMSGradient;
    int mMaximumIteration;
    int mIteration;

    double h(Eigen::VectorXd tau);
    double gradientH(int tauIndex);
    void updateTau();
    void solve();

};


#endif //RAISIM_DOUBLEBARSTEADYSTATECALCULATOR_H
