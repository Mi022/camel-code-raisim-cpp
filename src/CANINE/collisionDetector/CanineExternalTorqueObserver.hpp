//
// Created by cha on 22. 10. 4.
//

#ifndef RAISIM_CANINEEXTERNALTORQUEOBSERVER_HPP
#define RAISIM_CANINEEXTERNALTORQUEOBSERVER_HPP

#include "raisim/World.hpp"

class CanineExternalTorqueObserver
{
public:
    CanineExternalTorqueObserver();
    void UpdateBeta(double generalizedPositionOfJoint1, double generalizedVelocityOfJoint1, double generalizedPositionOfJoint2, double generalizedVelocityOfJoint2);
    Eigen::Matrix2d GetCoriloisMat(double generalizedPositionOfJoint1, double generalizedVelocityOfJoint1, double generalizedPositionOfJoint2, double generalizedVelocityOfJoint2);
    Eigen::Vector2d GetGravityMat(double generalizedPositionOfJoint1, double generalizedPositionOfJoint2);
    Eigen::Matrix2d GetMassMat(double generalizedPositionOfJoint1, double generalizedPositionOfJoint2);
    void UpdateState(raisim::VecDyn generalizedPosition, raisim::VecDyn generalizedVelocity, double torque1, double torque2);
    Eigen::Vector2d GetResidualVector();
    Eigen::Vector2d GetBeta();

private:
    const double mLink1 = 0.23;
    const double mLink2 = 0.23;
    const double mMass1 = 0.66;
    const double mMass2 = 0.18;
    const double mLinkC1 = 0.115;
    const double mLinkC2 = 0.115;
    const double mInertia1 = 0.0085;
    const double mInertia2 = 0.0058;
    const double mDT = 0.005;
    bool mbFirstRun;
    Eigen::Vector2d mMomentum;
    Eigen::Vector2d mMomentumPrev;
    Eigen::Vector2d mResidual;
    Eigen::Matrix2d mGain;
    Eigen::Vector2d mTempTorque;
    Eigen::Vector2d mGeneralizedPosition;
    Eigen::Vector2d mGeneralizedVelocity;
    Eigen::Vector2d mBeta;
};


#endif //RAISIM_CANINEEXTERNALTORQUEOBSERVER_HPP
