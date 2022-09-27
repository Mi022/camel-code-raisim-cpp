//
// Created by jy on 22. 7. 6.
//

#include "DistanceCalculator.h"
#include "iostream"

float DistanceCalculator::distance(Eigen::MatrixXd point1, Eigen::MatrixXd point2)
{
    float distanceValue;
    int size = point1.cols();
    for (int i = 0; i < size; i++)
    {
        distanceValue = distanceValue + (pow(point1(i) - point2(i), 2));
    }
    distanceValue = sqrt(distanceValue);

    return distanceValue;
}
