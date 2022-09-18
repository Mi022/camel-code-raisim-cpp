//
// Created by hs on 22. 7. 14.
//

#include "legtrajectory.h"
#include <iostream>

void legtrajectory::updateTrajectory(double currentTime, double timeDuration){
    mReferenceTime = currentTime;
    mTimeDuration = timeDuration;
}

/*
// 25 step period
double legtrajectory::getPositionTrajectory(double currentTime) {
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    int k = normalizedTime/0.125;
    normalizedTime -= double(k) * 0.125;
    return -25.6*pow(normalizedTime-0.0625,2)-0.2;
}
*/

/*
// 50 step period
double legtrajectory::getPositionTrajectory(double currentTime) {
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    int k = normalizedTime / 0.25;
    normalizedTime -= double(k) * 0.25;
    return -3.2 * pow(normalizedTime - 0.125, 2) - 0.25;
}*/

/*// 25 step period
double legtrajectory::getPositionTrajectory(double currenttime) {
    double normalizedTime = (currenttime - mReferenceTime) / mTimeDuration;
    normalizedTime -= floor(normalizedTime);
    return -0.4*pow(normalizedTime-0.5,2)-0.27;
}*/


double legtrajectory::factorial(double value){
    double result = 1.0;
    for(double i=1.0; i<=value; i++)
        result *= i;
    return result;
}

void legtrajectory::getPositionTrajectory(double currentTime) {
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    int scheduler = floor(normalizedTime);
    normalizedTime -= scheduler;
    sumX = 0.0;
    sumZ = 0.0;

    std::cout << scheduler << std::endl;
    if(scheduler%2 == 0)
    {
        double coeff = 0.0;
        for(int i=0; i<PNUM; i++){
            coeff = factorial(PNUM-1) / (factorial(i)* factorial(PNUM-1-i))
                    * pow(normalizedTime,i) * pow((1-normalizedTime), (PNUM-1-i));
            sumX +=  coeff * px[i];
            sumZ +=  coeff * pz[i];
        }
    }
    else
    {
        sumX = -0.25*normalizedTime+0.125;
        sumZ = 1.92*pow(sumX,2)-0.4;
        std::cout <<normalizedTime << "\t";
        std::cout <<sumX << "\t";
        std::cout <<sumZ << std::endl;

    }
}
