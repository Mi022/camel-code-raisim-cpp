//
// Created by user on 22. 6. 11.
//

#ifndef RAISIM_MIPLQRCONTROLLER_H
#define RAISIM_MIPLQRCONTROLLER_H

#include "include/CAMEL/Controller.h"
#include <Eigen/Core>

#define FORCE_DURATION 400
#define MAXIMUM_ITERATION 10000
#define TOLERANCE 0.001
#define MAX_POSITION_NOISE 0.001 //withstand 0.15
#define MAX_VELOCITY_NOISE 0.01 //withstand 1.4
#define MAX_MOTOR_VELOCITY_NOISE 0.001 //withstand 35.5
#define RANDOM_BOUNDARY 100

class MIPLQRController : public Controller {
public:
    MIPLQRController(Robot *robot) : Controller(robot) {
        mIteration = 0;
        setMatrix();
        setSNGain(100.0, 100.0, 100.0);
        setQGain(10.0, 100.0, 1.0);
        setRGain(1.0);
        mTorque.setZero();
        mS.setZero();
        findS();
        if(mIsSExist) {
            findK();
        }
        else{
            std::cout<<"there is no S"<<std::endl;
        }
    }

    void doControl() override;
    void updateState() override;
    void setTrajectory() override;
    void computeControlInput() override;
    void setControlInput() override;
    const Eigen::VectorXd &getTorque() const;

private:
    Eigen::VectorXd mTorque = Eigen::VectorXd(2);
    raisim::VecDyn mPosition = raisim::VecDyn(2);
    raisim::VecDyn mVelocity = raisim::VecDyn(2);
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
    void generateExternalForce();
    void addNoise();
};


#endif //RAISIM_MIPLQRCONTROLLER_H
