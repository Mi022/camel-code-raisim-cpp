//
// Created by jy on 22. 7. 8.
//

#ifndef RAISIM_LINEINTERPOLATION_H
#define RAISIM_LINEINTERPOLATION_H

#include "Eigen/Eigen"


class LineInterpolation
{

public:
    Eigen::VectorXd interpolation(float start, float end, int splitSize);
};


#endif //RAISIM_LINEINTERPOLATION_H
