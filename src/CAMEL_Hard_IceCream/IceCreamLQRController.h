//
// Created by hwayoung on 22. 9. 14.
//

#ifndef RAISIM_ICECREAMLQRCONTROLLER_H
#define RAISIM_ICECREAMLQRCONTROLLER_H

#include "include/CAMEL/Controller.h"
#include "IceCreamRobot.h"
#include <unsupported/Eigen/MatrixFunctions>

class IceCreamLQRController : public Controller {
public:
    IceCreamLQRController(IceCreamRobot *robot, double dT) : Controller(robot){
        torque = Eigen::VectorXd(robot->dim);
        desiredPosition = Eigen::VectorXd(robot->dim);
        desiredVelocity = Eigen::VectorXd(robot->dim);
        torque.setZero();
        desiredPosition.setZero();
        desiredVelocity.setZero();

        desiredPosition[1] = -90*3.141592/180.0;

        position = getRobot() -> getQ().e();
        velocity = getRobot() -> getQD().e();

        mDT = dT;
        mIteration = 0;
        setMatrix();
        setSNGain({100.0, 100.0, 100.0, 100.0});
        setQGain({100, 100, 100, 100});
        setRGain(1000);
        mS.setZero();
        findS();
        if(mIsSExist) {
            findK();
            std::cout<<"find K: "<<std::endl<<mK<<std::endl;
        }
        else{
            std::cout<<"there is no S"<<std::endl;
        }


    }

    Eigen::VectorXd torque;
    Eigen::VectorXd position;
    Eigen::VectorXd velocity;
    Eigen::VectorXd desiredPosition;
    Eigen::VectorXd desiredVelocity;

    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;

private:
    int mIteration;
    Eigen::Matrix4d mSN;
    Eigen::Matrix4d mQ;
    double mR;
    double mDT;
    Eigen::Matrix4d mA;
    Eigen::Matrix4d mS;
    Eigen::Matrix<double, 4, 1> mB;
    Eigen::Matrix<double, 1, 4> mK;
    Eigen::Matrix<double, 4, 1> mX;
    bool mIsSExist = true;
    double mTorqueLimit = 16.5;

    void setSNGain(Eigen::Vector4d D);
    void setQGain(Eigen::Vector4d D);
    void setRGain(double R);
    void setMatrix();
    void findS();
    void findK();
    void torqueLimit();
    void raisimDynamics();
    bool IsSEnough(Eigen::MatrixXd SN, Eigen::MatrixXd Snext);

    static const int mMaximumIteration;
    static const double mTolerance;
};


#endif //RAISIM_ICECREAMLQRCONTROLLER_H
