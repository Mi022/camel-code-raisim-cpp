//
// Created by hs on 22. 6. 24.
//

#include "SincurveTrajectoryGenerator.h"
#include "math.h"

void SincurveTrajectoryGenerator::updateTrajectory(double currentPosition, double currentTime, double amplitude, double frequency){
    mReferencePose = currentPosition;
    mReferenceTime = currentTime;
    mAmplitude = amplitude;
    mFrequency = frequency;
}

double SincurveTrajectoryGenerator::getPositionTrajectory(double currentTime) {
    double time = (currentTime - mReferenceTime);
    return mAmplitude*sin(2*PI*mFrequency*time - PI/2)+mReferencePose + mAmplitude;
}

double SincurveTrajectoryGenerator::getVelocityTrajectory(double currentTime) {
    double time = (currentTime - mReferenceTime);
    return 2*PI*mFrequency*mAmplitude*cos(2*PI*mFrequency*time - PI/2);
}