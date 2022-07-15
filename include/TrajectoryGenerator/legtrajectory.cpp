//
// Created by hs on 22. 7. 14.
//

#include "legtrajectory.h"
#include <iostream>

void legtrajectory::updateTrajectory(double currentPosition, double currentTime, double timeDuration){
    mReferencePose = currentPosition;
    mReferenceTime = currentTime;
    mTimeDuration = timeDuration;
}

double legtrajectory::getPositionTrajectory(double currentTime) {
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    int k = normalizedTime/0.125;
    normalizedTime -= double(k) * 0.125;
    return -25.6*pow(normalizedTime-0.0625,2)-0.2;
}
