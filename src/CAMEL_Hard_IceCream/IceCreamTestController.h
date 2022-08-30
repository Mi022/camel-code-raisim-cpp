//
// Created by hwayoung on 22. 8. 29.
//

#ifndef RAISIM_ICECREAMTESTCONTROLLER_H
#define RAISIM_ICECREAMTESTCONTROLLER_H

#include "include/CAMEL/Controller.h"     //include Controller class, trajectory generator class
#include "include/TrajectoryGenerator/CubicTrajectoryGenerator.h"
#include "IceCreamRobot.h"

class IceCreamTestController : public Controller {
public:
    IceCreamTestController(IceCreamRobot *robot, double dT) : Controller(robot) {
        torque = Eigen::VectorXd(robot->dim);
        position = getRobot() -> getQ().e();
        velocity = getRobot() -> getQD().e();

        desiredPosition = getRobot() -> getQ().e();
        desiredVelocity = getRobot() -> getQD().e();
        measuredAcc = Eigen::VectorXd(robot->dim).setZero();
        calculatedAcc = Eigen::VectorXd (robot->dim).setZero();

        mPreviousVelocity = getRobot() -> getQD().e();

        mDT = dT;
        mTrajectoryGenerator = CubicTrajectoryGenerator();
        mTrajectoryGenerator.updateTrajectory(position[1], -90*robot->deg2rad, robot->getWorldTime(), 3.0);
        mEndTime = robot->getWorldTime() + 1.0;

        mPGain = 1.0;
        mDGain = 0.4;

        mIra = parallelAxis(cubiodInertia(mRodMass, 0.03, mRodLength), mRodMass, mRodLength/2.0);
        mIBb = parallelAxis(cubiodInertia(mBodyMass, mBodyLength, 0.1), mBodyMass, mBodyLength/2.0);

    }

    Eigen::VectorXd torque;
    Eigen::VectorXd position;
    Eigen::VectorXd velocity;
    Eigen::VectorXd desiredPosition;
    Eigen::VectorXd desiredVelocity;
    Eigen::VectorXd measuredAcc;
    Eigen::VectorXd calculatedAcc;

    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;

private:
    double mPGain, mDGain;
    double mEndTime;
    double mDT;
    CubicTrajectoryGenerator mTrajectoryGenerator;
    Eigen::VectorXd mPreviousVelocity;
    double mRodLength = 0.35;
    double mRodMass = 1.2;
    double mBodyMass = 2.5;
    double mBodyLength = 0.3;
    double mIra, mIBa, mIICa, mIBb;


    void updateAcc();
    double cubiodInertia(double mass, double a, double b);
    double parallelAxis(double inertia, double mass, double length);
};



#endif //RAISIM_ICECREAMTESTCONTROLLER_H
