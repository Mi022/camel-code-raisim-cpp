//
// Created by hwayoung on 22. 9. 14.
//

#ifndef RAISIM_ICECREAMLQRCONTROLLER_H
#define RAISIM_ICECREAMLQRCONTROLLER_H

#include "include/CAMEL/Controller.h"
#include "IceCreamRobot.h"

class IceCreamLQRController : public Controller {
public:
    IceCreamLQRController(IceCreamRobot *robot) : Controller(robot){
        torque = Eigen::VectorXd(robot->dim);
        torque.setZero();
        position = getRobot() -> getQ().e();
        velocity = getRobot() -> getQD().e();

        mIteration = 0;
        setMatrix();
        setSNGain(100.0, 100.0, 100.0);
        setQGain(10.0, 100.0, 1.0);
        setRGain(1.0);
        mS.setZero();
        findS();
        if(mIsSExist) {
            findK();
        }
        else{
            std::cout<<"there is no S"<<std::endl;
        }


    }

    Eigen::VectorXd torque;
    Eigen::VectorXd position;
    Eigen::VectorXd velocity;

    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;

private:
    int mIteration;
    Eigen::Matrix3d mSN;
    Eigen::Matrix3d mQ;
    double mR;
    Eigen::Matrix3d mA;
    Eigen::Matrix3d mS;
    Eigen::Matrix<double, 3, 1> mB;
    Eigen::Matrix<double, 1, 3> mK;
    Eigen::Matrix<double, 3, 1> mX;
    bool mIsSExist = true;
    double mTorqueLimit = 3.5;

    void setSNGain(double SN11, double SN22, double SN33);
    void setQGain(double Q11, double Q22, double Q33);
    void setRGain(double R);
    void setMatrix();
    void findS();
    void findK();
    bool IsSEnough(Eigen::Matrix3d SN, Eigen::Matrix3d Snext);

};


#endif //RAISIM_ICECREAMLQRCONTROLLER_H
