//
// Created by user on 22. 6. 11.
//

#ifndef RAISIM_MIPILQRCONTROLLER_H
#define RAISIM_MIPILQRCONTROLLER_H

#include "include/CAMEL/Controller.h"
#include <Eigen/Core>

#define FORCE_DURATION 400
#define MAX_POSITION_NOISE 0.001 //withstand 0.04
#define MAX_VELOCITY_NOISE 0.01 //withstand 0.6
#define MAX_MOTOR_VELOCITY_NOISE 0.001 //withstand 35.5
#define RANDOM_BOUNDARY 100

class MIPILQRController : public Controller {
public:
    raisim::World *robotWorld;

    double torqueLimit = 3.5;

    MIPILQRController(Robot *robot, raisim::World *world, double dT) : Controller(robot) {
        robotWorld = world;
        mItertaion = 0;
        mDT = dT;
        setMatrix();
        setSNGain(100.0, 100.0, 100.0);
        setQGain(10.0, 100.0, 10.0);
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
    double getEstPlane() const;
    double getDesiredInclinePosition() const;

private:
    int mItertaion;
    Eigen::VectorXd mTorque = Eigen::VectorXd(3);
    raisim::VecDyn mPosition = raisim::VecDyn(3);
    raisim::VecDyn mVelocity = raisim::VecDyn(3);
    Eigen::VectorXd mInclineX = Eigen::VectorXd(2);
    Eigen::Matrix3d mSN;
    Eigen::Matrix3d mQ;
    double mR;
    Eigen::Matrix3d mA;
    Eigen::Matrix3d mS;
    Eigen::Matrix<double, 3, 1> mB;
    Eigen::Matrix<double, 1, 3> mK;
    Eigen::Matrix<double, 3, 1> mX;
    bool mIsSExist = true;
    double mEstPlane = 0.0;
    double mRodAcc = 0.0;
    double mReference = 0.0;
    double mDT;
    double mDesiredInclinePosition;
    double mDesiredInclineVelocity;

    void setSNGain(double SN11, double SN22, double SN33);
    void setQGain(double Q11, double Q22, double Q33);
    void setRGain(double R);
    void setMatrix();
    void findS();
    void findK();
    bool IsSEnough(Eigen::Matrix3d SN, Eigen::Matrix3d Snext);
    void generateExternalForce();
    void addNoise();
    void inclinePDcontrol();
    void setInclineUnitTrajectory(double desiredPosition, double desiredVelocity);
    void setInclineSinTrajectory(double amplitude, double period);
    void setInclineRisingTrajectory(double stepSize, double period);
    void calEstPlane();
    bool IsTorqueZero();
};


#endif //RAISIM_MIPILQRCONTROLLER_H
