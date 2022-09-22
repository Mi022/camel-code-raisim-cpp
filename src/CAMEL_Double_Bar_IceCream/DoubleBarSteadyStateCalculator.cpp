//
// Created by hwayoung on 22. 9. 22.
//

#include "DoubleBarSteadyStateCalculator.h"

double DoubleBarSteadyStateCalculator::h(Eigen::VectorXd tau) {
    Eigen::VectorXd QDDot = Eigen::VectorXd::Zero (mModel->qdot_size);
    RigidBodyDynamics::ForwardDynamics(*mModel, mDesiredPosition, mDesiredVelocity, tau, QDDot);
    return QDDot.transpose()*QDDot;
}

double DoubleBarSteadyStateCalculator::gradientH(int tauIndex) {
    Eigen::VectorXd deltaTau = tau;
    deltaTau[tauIndex] += mDelta;
    return (h(deltaTau) - h(tau))/mDelta;
}

void DoubleBarSteadyStateCalculator::updateTau() {
    Eigen::VectorXd nextTau = Eigen::VectorXd::Zero(tau.size());
    Eigen::VectorXd mGradient = Eigen::VectorXd::Zero(tau.size()-1);
    std::cout<<"hi_start update"<< std::endl;
    std::cout<<"gradientH(1): "<<gradientH(1)<< std::endl;
    std::cout<<"gradientH(2): "<<gradientH(2)<< std::endl;
    std::cout<<"tau[1] - mStepSize* gradientH(1): "<<tau[1] - mStepSize* gradientH(1)<< std::endl;
    std::cout<<"tau[2] - mStepSize* gradientH(2): "<<tau[2] - mStepSize* gradientH(2)<< std::endl;
    mGradient[0] = gradientH(1);
    mGradient[1] = gradientH(2);

    nextTau[1] = tau[1] - mStepSize* mGradient[0];
    nextTau[2] = tau[2] - mStepSize* mGradient[1];
    mRMSGradient = pow(mGradient.dot(mGradient) / mGradient.size() , 0.5);

    tau = nextTau;
    std::cout<<"tau: "<<tau<< std::endl;
}

void DoubleBarSteadyStateCalculator::solve() {
    while(true){
        std::cout<<"h(tau): "<<h(tau)<< std::endl;
        if(mRMSGradient < mTerminateCondition) {
            std::cout<<"Gradient Descent Optimizer is completed. (Terminate condition)"<<std::endl;
            break;
        }
        if(mIteration > mMaximumIteration) {
            std::cout<<"Gradient Descent Optimizer is failed. (Over maximum iteration)"<<std::endl;
            break;
        }
        std::cout<<"iteration: "<<mIteration<< std::endl;
        updateTau();
        mIteration++;

    }
}
