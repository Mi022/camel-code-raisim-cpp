//
// Created by hwayoung on 22. 9. 26.
//

#ifndef RAISIM_DOUBLEBARLQRCONTROLLER_HPP
#define RAISIM_DOUBLEBARLQRCONTROLLER_HPP

#include "include/CAMEL/Controller.h"
#include "DoubleBarRobot.h"
#include "DoubleBarStablePointFinder.h"
#include "DoubleBarSteadyStateCalculator.h"
#include "DoubleBarLinearEoMFinder.hpp"

class DoubleBarLQRController : public Controller
{
public:
    DoubleBarLQRController(DoubleBarRobot* robot, double dT, std::string urdfPath)
            : Controller(robot), mTorqueLimit(1000.0), mIsSExist(true)
    {
        // initialize
        mDesiredPosition = getRobot()->getQ().e();
        mDesiredVelocity = Eigen::VectorXd(robot->getDim()).setZero();
        mDesiredTorque = Eigen::VectorXd(robot->getDim()).setZero();
        mTorque = Eigen::VectorXd (mDesiredTorque.size());
        mX = Eigen::VectorXd(mDesiredPosition.size() + mDesiredVelocity.size());
        mTorque.setZero();

        //find stable point
        DoubleBarStablePointFinder stablePointFinder = DoubleBarStablePointFinder(mDesiredPosition, robot);
        stablePointFinder.FindStableState();
        std::cout << "optimal stable point: " << std::endl << stablePointFinder.getPosition() << std::endl;
        mDesiredPosition = stablePointFinder.getPosition();

        //calculate torque in stable point
        DoubleBarSteadyStateCalculator steadyStateCalculator = DoubleBarSteadyStateCalculator(urdfPath, mDesiredPosition, mDesiredVelocity);
        steadyStateCalculator.SolveTorque();
        std::cout << "optimal tau: " << steadyStateCalculator.getTau() << std::endl;
        mDesiredTorque = steadyStateCalculator.getTau();

        //find linear EoM
        DoubleBarLinearEoMFinder linearEoMFinder = DoubleBarLinearEoMFinder(urdfPath, mDesiredPosition, mDesiredVelocity, mDesiredTorque, dT);
        linearEoMFinder.FindAB();
        mA = linearEoMFinder.GetA();
        mB = linearEoMFinder.GetB();
        std::cout << "mA: " << mA << std::endl;
        std::cout << "mB: " << mB << std::endl;

        //LQR_find k
        Eigen::VectorXd SN = Eigen::VectorXd (mX.size());
        Eigen::VectorXd Q = Eigen::VectorXd (mX.size());
        Eigen::VectorXd R = Eigen::VectorXd (mX.size());
        SN << 100.0, 100.0, 100.0, 100.0 , 100.0, 100.0;
        Q << 100.0, 100.0, 100.0, 100.0 , 100.0, 100.0;
        R << 1000.0, 100.0;
        setSNGain(SN);
        setQGain(Q);
        setRGain(R);
        findS();
        if (mIsSExist)
        {
            findK();
            std::cout << "find K: " << std::endl << mK << std::endl;
        }
        else
        {
            std::cout << "there is no S" << std::endl;
        }

    }
    const Eigen::MatrixXd& GetX() const;

    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;

private:
    void setSNGain(Eigen::VectorXd D);
    void setQGain(Eigen::VectorXd D);
    void setRGain(Eigen::VectorXd D);
    void findS();
    void findK();
    bool bIsSEnough(Eigen::MatrixXd SN, Eigen::MatrixXd Snext);
    void torqueLimit();

    Eigen::VectorXd mDesiredPosition;
    Eigen::VectorXd mDesiredVelocity;
    Eigen::VectorXd mDesiredTorque;
    Eigen::VectorXd mTorque;
    Eigen::MatrixXd mA;
    Eigen::MatrixXd mB;
    Eigen::MatrixXd mQ;
    Eigen::MatrixXd mR;
    Eigen::MatrixXd mSN;
    Eigen::MatrixXd mS;
    Eigen::MatrixXd mK;
    Eigen::VectorXd mX;
    double mTorqueLimit;
    bool mIsSExist;

    static const int mMaximumIteration;
    static const double mTolerance;

};


#endif //RAISIM_DOUBLEBARLQRCONTROLLER_HPP
