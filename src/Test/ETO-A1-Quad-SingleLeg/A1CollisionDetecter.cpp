//
// Created by cha on 22. 9. 14.
//

#include "A1CollisionDetecter.h"

A1CollisionDetecter::A1CollisionDetecter()
{
    mMomentum << 0, 0;
    mMomentumPrev << 0, 0;
    mResidual << 0, 0;
    mGain << 100, 0, 0, 100;
    mTempTorque << 0, 0;
    mGeneralizedPosition << 0, 0;
    mGeneralizedVelocity << 0, 0;
    mbFirstRun << 1;
}

/**
 * Get beta matrix.
 *
 * @return 2x1 beta matrix
 */
Eigen::Vector2d A1CollisionDetecter::GetBeta()
{
    return mBeta;
};

/**
     *
     * @param q1    the position of first joint
     * @param dq1   the velocity of first joint
     * @param q2    the position of second joint
     * @param dq2   the velocity of second joint
     * @return      2x2 corilois matrix
     */
Eigen::Matrix2d A1CollisionDetecter::GetCoriloisMat(double q1, double dq1, double q2, double dq2)
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
Eigen::Vector2d A1CollisionDetecter::GetGravityMat(double q1, double q2)
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
Eigen::Matrix2d A1CollisionDetecter::GetMassMat(double q1, double q2)
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

/**
 *
 * @return 2x1 rsidual vector.
 */
Eigen::Vector2d A1CollisionDetecter::GetResidualVector()
{
    return mResidual;
};



/**
     *
     * @param q1    the position of first joint
     * @param dq1   the velocity of first joint
     * @param q2    the position of second joint
     * @param dq2   the velocity of second joint
     * @return      2x1 beta matrix which consist of gravity matrix and transpose of coriolis matrix
     */
void A1CollisionDetecter::UpdateBeta(double q1, double dq1, double q2, double dq2)
{
    Eigen::VectorXd dqMat = Eigen::VectorXd(2);
    dqMat[0] = dq1;
    dqMat[1] = dq2;
    mBeta = GetGravityMat(q1, q2) - GetCoriloisMat(q1, dq1, q2, dq2).transpose() * dqMat;
};

/**
 * Update robot's state and calculate residual vector, which isn't zero when the external torque occur.
 *
 * @param generalizedPosition generalized position of the robot
 * @param generalizedVelocity generalized velocity of the robot
 * @param torque calculated torque to activate the robot's joints
 */
void A1CollisionDetecter::UpdateState(raisim::VecDyn generalizedPosition, raisim::VecDyn generalizedVelocity, Eigen::Vector3d torque)
{
    mGeneralizedPosition[0] = generalizedPosition[1];
    mGeneralizedPosition[1] = generalizedPosition[2];
    mGeneralizedVelocity[0] = generalizedVelocity[1];
    mGeneralizedVelocity[1] = generalizedVelocity[2];
    mTempTorque[0] = torque[1];
    mTempTorque[1] = torque[2];
    if (mbFirstRun == true)
    {
        mMomentumPrev[0] = 0;
        mMomentumPrev[1] = 0;
        mResidual[0] = 0;
        mResidual[1] = 0;
        mbFirstRun = false;
    }
    mMomentum = mMomentumPrev + mTempTorque * mDT - mBeta * mDT + mResidual * mDT;
    mResidual = mGain * (-mMomentum + this->GetMassMat(mGeneralizedPosition[0], mGeneralizedPosition[1]) * mGeneralizedVelocity);
    mMomentumPrev = mMomentum;
};

