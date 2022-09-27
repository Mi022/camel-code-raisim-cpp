//
// Created by jy on 22. 7. 6.
//

#ifndef RAISIM_DISTANCECALCULATOR_H
#define RAISIM_DISTANCECALCULATOR_H

#include "Eigen/Eigen"
#include <cmath>

class DistanceCalculator
{
public:
    float distance(Eigen::MatrixXd point1, Eigen::MatrixXd point2);

};


#endif //RAISIM_DISTANCECALCULATOR_H
