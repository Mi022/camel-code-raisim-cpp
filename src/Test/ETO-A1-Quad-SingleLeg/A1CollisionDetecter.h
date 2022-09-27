//
// Created by cha on 22. 9. 14.
//

#ifndef RAISIM_A1COLLISIONDETECTER_H
#define RAISIM_A1COLLISIONDETECTER_H

#include "raisim/World.hpp"

class A1CollisionDetecter
{
public:
    A1CollisionDetecter();
    Eigen::Vector2d Beta(double q1, double dq1, double q2, double dq2);
    Eigen::Matrix2d CoriloisMat(double q1, double dq1, double q2, double dq2);
    Eigen::Vector2d GravityMat(double q1, double q2);
    Eigen::Matrix2d MassMat(double q1, double q2);

private:
    const double mLink1 = 0.2;
    const double mLink2 = 0.22;
    const double mMass1 = 1.013;
    const double mMass2 = 0.172;
    const double mLinkC1 = 0.027;
    const double mLinkC2 = 0.134;
    const double mInertia1 = 0.0085;
    const double mInertia2 = 0.0058;

};

#endif //RAISIM_A1COLLISIONDETECTER_H
