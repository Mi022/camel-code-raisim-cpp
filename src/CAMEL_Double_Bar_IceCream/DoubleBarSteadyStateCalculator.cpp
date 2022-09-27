//
// Created by hwayoung on 22. 9. 22.
//

#include "DoubleBarSteadyStateCalculator.h"

void DoubleBarSteadyStateCalculator::getModelFromURDF(std::string urdfPath) {
    mModel = new RigidBodyDynamics::Model();
    bool modelLoaded = RigidBodyDynamics::Addons::URDFReadFromFile(urdfPath.c_str(), mModel, false);
    std::cout<<"check: "<<modelLoaded<<std::endl;
}

double DoubleBarSteadyStateCalculator::h(Eigen::VectorXd tau)
{
    RigidBodyDynamics::ForwardDynamics(*mModel, mDesiredPosition, mDesiredVelocity, tau, mQDDot);
    return mQDDot.transpose()*mQDDot;
}

double DoubleBarSteadyStateCalculator::gradientH(int tauIndex)
{
    Eigen::VectorXd deltaTau = mTau;
    deltaTau[tauIndex] += mDelta;
    return (h(deltaTau) - h(mTau))/mDelta;
}

void DoubleBarSteadyStateCalculator::updateTau()
{
    Eigen::VectorXd nextTau = Eigen::VectorXd::Zero(mTau.size());
    Eigen::VectorXd mGradient = Eigen::VectorXd::Zero(mTau.size()-1);
    mGradient[0] = gradientH(1);
    mGradient[1] = gradientH(2);

    nextTau[1] = mTau[1] - mStepSize* mGradient[0];
    nextTau[2] = mTau[2] - mStepSize* mGradient[1];
    mRMSGradient = pow(mGradient.dot(mGradient) / mGradient.size() , 0.5);

    mTau = nextTau;
}

void DoubleBarSteadyStateCalculator::SolveTorque()
{
    while(true){
        if(mRMSGradient < mTerminateCondition) {
            std::cout<<"Gradient Descent Optimizer is completed. (Terminate condition)"<<std::endl;
            break;
        }
        if(mIteration > mMaximumIteration) {
            std::cout<<"Gradient Descent Optimizer is failed. (Over maximum iteration)"<<std::endl;
            break;
        }
        updateTau();
        mIteration++;
    }
}

const Eigen::VectorXd &DoubleBarSteadyStateCalculator::getTau() const {
    return mTau;
}
