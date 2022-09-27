//
// Created by hwayoung on 22. 9. 26.
//

#include "DoubleBarLinearEoMFinder.hpp"

DoubleBarLinearEoMFinder::DoubleBarLinearEoMFinder(std::string urdfPath, Eigen::VectorXd desiredPosition, Eigen::VectorXd desiredVelocity, Eigen::VectorXd desiredTorque, double dT)
    : mDesiredPosition(desiredPosition)
    , mDesiredVelocity(desiredVelocity)
    , mDesiredTorque(desiredTorque)
    , mDT(dT)
{
    mModel = new RigidBodyDynamics::Model();
    getModelFromURDF(urdfPath);
}

const Eigen::MatrixXd& DoubleBarLinearEoMFinder::GetA() const
{
    return mA;
}

const Eigen::MatrixXd& DoubleBarLinearEoMFinder::GetB() const
{
    return mB;
}

void DoubleBarLinearEoMFinder::FindAB()
{
    generateAc();
    generateBc();
    generateABd();
}

void DoubleBarLinearEoMFinder::getModelFromURDF(std::string urdfPath)
{
    bool modelLoaded = RigidBodyDynamics::Addons::URDFReadFromFile(urdfPath.c_str(), mModel, false);
    std::cout << "check: " << modelLoaded << std::endl;
    mQDDot = Eigen::VectorXd::Zero(mModel->qdot_size);
    RigidBodyDynamics::ForwardDynamics(*mModel, mDesiredPosition, mDesiredVelocity, mDesiredTorque, mQDDot);
}

Eigen::MatrixXd DoubleBarLinearEoMFinder::computePartialDiff(eFactor factor)
{
    Eigen::MatrixXd partialDiff;
    Eigen::VectorXd QDDot_delta = Eigen::VectorXd::Zero(mModel->qdot_size);

    switch (factor)
    {
    case eFactor::Q:
        partialDiff = Eigen::MatrixXd(mModel->qdot_size, mModel->q_size);
        for (int idx = 0; idx < partialDiff.cols(); idx++)
        {
            Eigen::VectorXd Q_delta = Eigen::VectorXd::Zero(partialDiff.cols());
            Q_delta = mDesiredPosition;
            Q_delta[idx] += mDelta;
            RigidBodyDynamics::ForwardDynamics(*mModel, Q_delta, mDesiredVelocity, mDesiredTorque, QDDot_delta);
            partialDiff.block(0, idx, QDDot_delta.rows(), QDDot_delta.cols()) = (QDDot_delta - mQDDot) / mDelta;
        }
        break;
    case eFactor::Q_DOT:
        partialDiff = Eigen::MatrixXd(mModel->qdot_size, mModel->qdot_size);
        for (int idx = 0; idx < partialDiff.cols(); idx++)
        {
            Eigen::VectorXd QDot_delta = Eigen::VectorXd::Zero(partialDiff.cols());
            QDot_delta = mDesiredVelocity;
            QDot_delta[idx] += mDelta;
            RigidBodyDynamics::ForwardDynamics(*mModel, mDesiredPosition, QDot_delta, mDesiredTorque, QDDot_delta);
            partialDiff.block(0, idx, QDDot_delta.rows(), QDDot_delta.cols()) = (QDDot_delta - mQDDot) / mDelta;
        }
        break;
    case eFactor::TAU:
        partialDiff = Eigen::MatrixXd(mModel->qdot_size, mModel->q_size - 1);
        for (int idx = 0; idx < partialDiff.cols(); idx++)
        {
            Eigen::VectorXd tau_delta = Eigen::VectorXd::Zero(partialDiff.cols());
            tau_delta = mDesiredTorque;
            tau_delta[idx + 1] += mDelta;
            RigidBodyDynamics::ForwardDynamics(*mModel, mDesiredPosition, mDesiredVelocity, tau_delta, QDDot_delta);
            partialDiff.block(0, idx, QDDot_delta.rows(), QDDot_delta.cols()) = (QDDot_delta - mQDDot) / mDelta;
        }
        break;
    default:
        break;
    }
    return partialDiff;
}

void DoubleBarLinearEoMFinder::generateAc()
{
    mA = Eigen::MatrixXd(mModel->q_size + mModel->qdot_size, 2 * mModel->qdot_size);
    mA.block(0, 0, mModel->q_size, mModel->qdot_size) = Eigen::MatrixXd(mModel->q_size, mModel->qdot_size).setZero();
    mA.block(0, mModel->qdot_size, mModel->q_size, mModel->qdot_size) = Eigen::MatrixXd(mModel->q_size, mModel->qdot_size).setIdentity();
    mA.block(mModel->q_size, 0, mModel->qdot_size, mModel->qdot_size) = computePartialDiff(eFactor::Q);
    mA.block(mModel->q_size, mModel->qdot_size, mModel->qdot_size, mModel->qdot_size) = computePartialDiff(eFactor::Q_DOT);
}

void DoubleBarLinearEoMFinder::generateBc()
{
    mB = Eigen::MatrixXd(2 * mModel->qdot_size, mModel->q_size - 1);
    mB.block(0, 0, mModel->qdot_size, mModel->q_size - 1) = Eigen::MatrixXd(mModel->qdot_size, mModel->q_size - 1).setZero();
    mB.block(mModel->qdot_size, 0, mModel->qdot_size, mModel->q_size - 1) = computePartialDiff(eFactor::TAU);
}

void DoubleBarLinearEoMFinder::generateABd()
{
    Eigen::MatrixXd ABc = Eigen::MatrixXd (mA.cols()+mB.cols(), mA.cols()+mB.cols());
    ABc.setZero();
    ABc.block(0, 0, mA.rows(), mA.cols()) = mA;
    ABc.block(0, mA.cols(),mB.rows(), mB.cols()) = mB;
    ABc = ABc*mDT;
    ABc = ABc.exp(); // y'=My => y(t)=exp(M)y(0)
    mA = ABc.block(0, 0, mA.rows(), mA.cols());
    mB = ABc.block(0, mA.cols(),mB.rows(), mB.cols());
}

void DoubleBarLinearEoMFinder::testAB()
{
    Eigen::VectorXd Q;
    Q = mDesiredPosition;
    Eigen::VectorXd X = Eigen::VectorXd(mModel->q_size+mModel->qdot_size);
    X.block(0, 0, mModel->q_size, 1) = mDesiredPosition;
    X.block(mModel->q_size, 0, mModel->qdot_size, 1) = mDesiredVelocity;
    X.block(0, 0, mModel->q_size, 1) = X.block(0, 0, mModel->q_size, 1) - mDesiredPosition;
    Eigen::VectorXd U = mDesiredTorque.block(1,0,mModel->qdot_size-1, 1);
    U = U - mDesiredTorque.block(1,0,mModel->qdot_size-1, 1);
    double plus = 1e-3;

    Eigen::VectorXd QDDot_lin;

    Q[0] = Q[0] + plus;
    RigidBodyDynamics::ForwardDynamics(*mModel, Q, mDesiredVelocity, mDesiredTorque, mQDDot);
    std::cout << "RBDL_QDDot: " << std::endl << mQDDot << std::endl;
    Q[0] = Q[0] - plus;

    X[0] = X[0] + plus;
    QDDot_lin = mA * X + mB * U;
    std::cout << "Linear_QDDot: " << std::endl << QDDot_lin << std::endl;
    X[0] = X[0] - plus;
}

const double DoubleBarLinearEoMFinder::mDelta = 1e-3;
