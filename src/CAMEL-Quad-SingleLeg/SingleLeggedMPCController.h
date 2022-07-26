//
// Created by pnuca on 2022-06-15.
//

#ifndef RAISIM_SINGLELEGGEDMPCCONTROLLER_H
#define RAISIM_SINGLELEGGEDMPCCONTROLLER_H

#include "include/CAMEL/Controller.h"
#include "include/TrajectoryGenerator/QuinticTrajectoryGenerator.h"
#include "include/TrajectoryGenerator/SincurveTrajectoryGenerator.h"

class SingleLeggedMPCController : public Controller {
public:
    Eigen::VectorXd torque = Eigen::VectorXd(3);

    raisim::VecDyn position = raisim::VecDyn(3);
    raisim::VecDyn velocity = raisim::VecDyn(3);
    Eigen::VectorXd positionError = Eigen::VectorXd(2);
    Eigen::VectorXd velocityError = Eigen::VectorXd(2);
    Eigen::VectorXd desiredJointPosition = Eigen::VectorXd(2);
    Eigen::VectorXd desiredJointPosition_past = Eigen::VectorXd(2);
    Eigen::VectorXd desiredJointVelocity = Eigen::VectorXd(2);

    double desiredPosition;
    double desiredVelocity;
    double desiredAcceleration;
    double calculatedForce = 0.0;

    // jacobian
    double dz_dth1 = 0.0;
    double dz_dth2 = 0.0;

    double torqueLimit = 13.0;

    SingleLeggedMPCController(Robot *robot, double dT) : Controller(robot) {
            updateState();
            mTrajectoryGenerator.updateTrajectory(position[0], getRobot()->getWorldTime(), 0.1,0.2);
            torque[0] = 0.0;
            mDT = dT;
            mMaximumIteration = 100;
            mTerminateCondition = 1e-7;
            mDelta = 1e-3;
            mStepSize = 100.0 / mDT / mDT;
            mA << 1.0, mDT, 0.0, 1.0;
            mB << 0.0, mDT;
            mQ << 1.5, 0.0, 0.0, 1e-3;
            mR << 1e-3 * mDT * mDT;
            mIteration = 0;
            mForce.setOnes();
            mForce = mForce * mLumpedMass * mGravity * (-1);
    }
    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;
    void solve();
    void updateMPCStates();
    void updateMPCStatesTemp(Eigen::VectorXd force);
    void computeGradient();
    void updateForces();
    void resetMPCVariables();
    bool isTerminateCondition();
    double objectiveFunction(Eigen::VectorXd force);

private:
//    QuinticTrajectoryGenerator mTrajectoryGenerator;
    SincurveTrajectoryGenerator mTrajectoryGenerator;
    double mLumpedMass = 2.009;
    double mGravity = -9.81;
    double mDT;

    int mMPCHorizon = 5;
    int mIteration;
    int mMaximumIteration;
    double mTerminateCondition;
    double mDelta;
    double mStepSize;
    double mInitialPosition;
    double mInitialVelocity;
//    double mInitialForce = 19.7083;
    double mRMSGradient;
    Eigen::MatrixXd mTrajectorySequence = Eigen::MatrixXd(2, mMPCHorizon);
    Eigen::MatrixXd mA = Eigen::MatrixXd(2, 2);
    Eigen::VectorXd mB = Eigen::VectorXd(2);
    Eigen::MatrixXd mQ = Eigen::MatrixXd(2, 2);
    Eigen::VectorXd mR = Eigen::VectorXd(1);
    Eigen::VectorXd mGradient = Eigen::VectorXd(mMPCHorizon);
    Eigen::MatrixXd mNextStates = Eigen::MatrixXd(2,mMPCHorizon);
    Eigen::MatrixXd mNextStatesTemp = Eigen::MatrixXd(2,mMPCHorizon);
    Eigen::VectorXd mForce = Eigen::VectorXd(mMPCHorizon);
    Eigen::VectorXd mForceTemp = Eigen::VectorXd(mMPCHorizon);

};


#endif //RAISIM_SINGLELEGGEDMPCCONTROLLER_H
