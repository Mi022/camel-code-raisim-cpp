//
// Created by hs on 22. 6. 24.
//

#include "SincurveTrajectoryGenerator.h"
#include "math.h"

void SincurveTrajectoryGenerator::updateTrajectory(double currentPosition, double currentTime, double timeDuration) {
    mReferencePose = currentPosition;
    mReferenceTime = currentTime;
    mTimeDuration = timeDuration;
}

double SincurveTrajectoryGenerator::getPositionTrajectory(double currentTime) {
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    return 0.05*sin(normalizedTime*2*3.14)+mReferencePose;
}

double SincurveTrajectoryGenerator::getVelocityTrajectory(double currentTime) {
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    return 0.05*cos(normalizedTime*2*3.14)*2*3.14;
}