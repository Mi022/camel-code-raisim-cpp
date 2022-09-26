//
// Created by hwayoung on 22. 9. 25.
//

#include "DoubleBarStablePointFinder.h"

double DoubleBarStablePointFinder::L(Eigen::VectorXd position)
{
    mRobot->robot->setGeneralizedCoordinate(position);
    Eigen::VectorXd COMPosition;
    COMPosition = mRobot->robot->getCOM().e();
    return COMPosition[0]*COMPosition[0];
}

double DoubleBarStablePointFinder::gradientL(int positionIndex)
{
    Eigen::VectorXd deltaPosition = mPosition;
    deltaPosition[positionIndex] += mDelta;
    return (L(deltaPosition) - L(mPosition))/mDelta;
}

void DoubleBarStablePointFinder::updatePosition()
{
    Eigen::VectorXd nextPosition = Eigen::VectorXd::Zero(mPosition.size());
    Eigen::VectorXd mGradient = Eigen::VectorXd::Zero(mPosition.size());
    mGradient[0] = gradientL(0);
    mGradient[1] = gradientL(1);
    mGradient[2] = gradientL(2);

    nextPosition[0] = mPosition[0] - mStepSize* mGradient[0];
    nextPosition[1] = mPosition[1] - mStepSize* mGradient[1];
    nextPosition[2] = mPosition[2] - mStepSize* mGradient[2];
    mRMSGradient = pow(mGradient.dot(mGradient) / mGradient.size() , 0.5);

    mPosition = nextPosition;
}

void DoubleBarStablePointFinder::FindStableState()
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
        updatePosition();
        mIteration++;
    }
}

const Eigen::VectorXd &DoubleBarStablePointFinder::getPosition() const {
    return mPosition;
}
