//
// Created by hwayoung on 22. 9. 25.
//

#ifndef RAISIM_DOUBLEBARSTABLEPOINTFINDER_H
#define RAISIM_DOUBLEBARSTABLEPOINTFINDER_H

#include "DoubleBarRobot.h"


class DoubleBarStablePointFinder{
public:
    DoubleBarStablePointFinder(Eigen::VectorXd desiredPosition, DoubleBarRobot* robot) : mPosition(desiredPosition), mRobot(robot)
    {
        mIteration = 0;
        mDelta = 1e-3;
        mStepSize = 8*1e-2;
        mTerminateCondition = 1e-4;
        mMaximumIteration = 1000;
        mRMSGradient = 10000;

        findStableState();
    }
    const Eigen::VectorXd &getPosition() const;

private:
    void findStableState();
    double L(Eigen::VectorXd position);
    double gradientL(int tauIndex);
    void updatePosition();

    DoubleBarRobot* mRobot;
    Eigen::VectorXd mPosition;
    double mDelta;
    double mStepSize;
    double mTerminateCondition;
    double mRMSGradient;
    int mMaximumIteration;
    int mIteration;
};


#endif //RAISIM_DOUBLEBARSTABLEPOINTFINDER_H
