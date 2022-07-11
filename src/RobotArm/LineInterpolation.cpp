//
// Created by jy on 22. 7. 8.
//

#include "LineInterpolation.h"

Eigen::VectorXd LineInterpolation::interpolation(float start, float end, int splitSize) {
    Eigen::VectorXd result = Eigen::VectorXd(splitSize+1);
    result[0] = start;
    float currentValue = start;
    for(int i = 1;i<=splitSize;i++){
        currentValue = currentValue + (abs(end - start))/splitSize;
        result[i] = currentValue;
    }
    return result;
}