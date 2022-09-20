//
// Created by cha on 22. 9. 19.
//

#include "A1CollisionDetecterNoInertia.h"

void A1CollisionDetecterNoInertia::changeConfig(double thighLength, double calfLength, double thighMass, double calfMass, double thighComLength, double calfComLength) {
    mlink1 = thighLength;
    mmass1 = thighMass;
    mlink2 = calfLength;
    mmass2 = calfMass;
};

Eigen::Vector2d A1CollisionDetecterNoInertia::Beta(double q1, double dq1, double q2, double dq2) {
    Eigen::VectorXd dqMat = Eigen::VectorXd(2);
    Eigen::Vector2d ans;
    dqMat[0] = dq1;
    dqMat[1] = dq2;
    ans = GravityMat(q1,q2) - CoriloisMat(q1, dq1, q2, dq2).transpose() * dqMat;
    return ans;
};

Eigen::Matrix2d A1CollisionDetecterNoInertia::CoriloisMat(double q1, double dq1, double q2, double dq2) {
    Eigen::Matrix2d coriolisMat;
    coriolisMat(0, 0) = -mmass2 * mlink1 * mlink2 * std::sin(q2) * dq2;
    coriolisMat(0, 1) = -mmass2 * mlink1 * mlink2 * std::sin(q2) * dq1 - mmass2 * mlink1 * mlink2 * std::sin(q2) * dq2;
    coriolisMat(1, 0) = mmass2 * mlink1 * mlink2 * std::sin(q2) * dq1;
    coriolisMat(1, 1) = 0;
    return coriolisMat;
};

Eigen::Vector2d A1CollisionDetecterNoInertia::GravityMat(double q1, double q2) {
    Eigen::Vector2d gravityMat;
    gravityMat[0] = (mmass1*mlink1 + mmass2*mlink1)*9.8*std::cos(q1) + mmass2*mlink2*9.8*std::sin(q1+q2);
    gravityMat[1] = mmass2*mlink2*9.8*std::cos(q1+q2);
    return gravityMat;
};

Eigen::Matrix2d A1CollisionDetecterNoInertia::MassMat(double q1, double q2) {
    Eigen::Matrix2d massMat;
    massMat(0,0) = mmass1 * mlink1 * mlink1 + mmass2 * (mlink1 * mlink1 + 2 * mlink1 * mlink2 * std::cos(q2) + mlink2 * mlink2);
    massMat(0,1) = mmass2 * (mlink2 * mlink2 + mlink1 * mlink2 * std::cos(q2));
    massMat(1,0) = mmass2 * (mlink2 * mlink2 + mlink1 * mlink2 * std::cos(q2));
    massMat(1,1) = mmass2 * mlink2 * mlink2;
    return massMat;
};