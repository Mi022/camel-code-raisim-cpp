//
// Created by cha on 22. 9. 14.
//

#include "A1CollisionDetecter.h"

A1CollisionDetecter::A1CollisionDetecter()
{
}

/**
     *
     * @param q1    the position of first joint
     * @param dq1   the velocity of first joint
     * @param q2    the position of second joint
     * @param dq2   the velocity of second joint
     * @return      2x1 beta matrix which consist of gravity matrix and transpose of coriolis matrix
     */
Eigen::Vector2d A1CollisionDetecter::Beta(double q1, double dq1, double q2, double dq2)
{
    Eigen::VectorXd dqMat = Eigen::VectorXd(2);
    Eigen::Vector2d ans;
    q1 = -q1 - 3.141592 / 2;
    q2 = -q2;
    dqMat[0] = dq1;
    dqMat[1] = dq2;
    ans = GravityMat(q1, q2) - CoriloisMat(q1, dq1, q2, dq2).transpose() * dqMat;
    return ans;
};

/**
     *
     * @param q1    the position of first joint
     * @param dq1   the velocity of first joint
     * @param q2    the position of second joint
     * @param dq2   the velocity of second joint
     * @return      2x2 corilois matrix
     */
Eigen::Matrix2d A1CollisionDetecter::CoriloisMat(double q1, double dq1, double q2, double dq2)
{
    Eigen::Matrix2d coriolisMat;
    q1 = -q1 - 3.141592 / 2;
    q2 = -q2;
    coriolisMat(0, 0) = -mMass2 * mLink1 * mLinkC2 * std::sin(q2) * dq2;
    coriolisMat(0, 1) = -mMass2 * mLink1 * mLinkC2 * std::sin(q2) * dq1 - mMass2 * mLink1 * mLinkC2 * std::sin(q2) * dq2;
    coriolisMat(1, 0) = mMass2 * mLink1 * mLinkC2 * std::sin(q2) * dq1;
    coriolisMat(1, 1) = 0;
    return coriolisMat;
};

/**
     *
     * @param q1    the position of first joint
     * @param q2    the position of second joint
     * @return      2x1 gravity matrix
     */
Eigen::Vector2d A1CollisionDetecter::GravityMat(double q1, double q2)
{
    Eigen::Vector2d gravityMat;
    gravityMat[0] = (mMass1 * mLinkC1 + mMass2 * mLink1) * 9.8 * std::sin(q1) + mMass2 * mLinkC2 * 9.8 * std::sin(q1 + q2);
    gravityMat[1] = mMass2 * mLinkC2 * 9.8 * std::sin(q1 + q2);
    return gravityMat;
};

/**
     *
     * @param q1    the position of first joint
     * @param q2    the position of second joint
     * @return      2x2 mass matrix
     */
Eigen::Matrix2d A1CollisionDetecter::MassMat(double q1, double q2)
{
    Eigen::Matrix2d massMat;
    q1 = -q1 - 3.141592 / 2;
    q2 = -q2;
    massMat(0, 0) = mMass1 * mLinkC1 * mLinkC1 + mMass2 * (mLink1 * mLink1 + 2 * mLink1 * mLinkC2 * std::cos(q2) + mLinkC2 * mLinkC2) + mInertia1 + mInertia2;
    massMat(0, 1) = mMass2 * (mLinkC2 * mLinkC2 + mLink1 * mLinkC2 * std::cos(q2)) + mInertia2;
    massMat(1, 0) = mMass2 * (mLinkC2 * mLinkC2 + mLink1 * mLinkC2 * std::cos(q2)) + mInertia2;
    massMat(1, 1) = mMass2 * mLinkC2 * mLinkC2 + mInertia2;
    return massMat;
};