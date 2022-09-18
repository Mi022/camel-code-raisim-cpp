//
// Created by hs on 22. 7. 14.
//

#include "Trajectory.h"
#include <iostream>

void LegTrajectory::updateTrajectory(double currentTime, double timeDuration){
    mReferenceTime = currentTime;
    mTimeDuration = timeDuration;
}

/*
// 25 step period
double LegTrajectory::getPositionTrajectory(double currentTime) {
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    int k = normalizedTime/0.125;
    normalizedTime -= double(k) * 0.125;
    return -25.6*pow(normalizedTime-0.0625,2)-0.27;
}
*/

double LegTrajectory::factorial(double value){
    double result = 1.0;
    for(double i=1.0; i<=value; i++)
        result *= i;
    return result;
}

void LegTrajectory::getPositionTrajectory(double currentTime) {
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    normalizedTime -= floor(normalizedTime);
    sumX = 0.0;
    sumZ = 0.0;

    double coeff = 0.0;
    for(int i=0; i<PNUM; i++){
        coeff = factorial(PNUM-1) / (factorial(i)* factorial(PNUM-1-i))
                * pow(normalizedTime,i) * pow((1-normalizedTime), (PNUM-1-i));
        sumX +=  coeff * px[i];
        sumZ +=  coeff * pz[i];
    }

}
