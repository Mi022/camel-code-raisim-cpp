//
// Created by user on 22. 6. 11.
//

#ifndef RAISIM_MIPILQRCONTROLLER_H
#define RAISIM_MIPILQRCONTROLLER_H

#include "include/CAMEL/Controller.h"
#include <Eigen/Core>

class MIPILQRController : public Controller {
public:
    Eigen::VectorXd torque = Eigen::VectorXd(3);
    raisim::VecDyn position = raisim::VecDyn(3);
    raisim::VecDyn velocity = raisim::VecDyn(3);
    raisim::World *robotWorld;

//    raisim::Vec<3> externalForce;
//    raisim::Vec<3> forcePosition;

    double torqueLimit = 3.5;

    MIPILQRController(Robot *robot, raisim::World *world) : Controller(robot) {
        robotWorld = world;
        i = 0;
        setMatrix();
        setSNGain(100.0, 100.0, 100.0);
        setQGain(10.0, 100.0, 10.0);
        setRGain(1.0);
        torque.setZero();
        S.setZero();
        findS();
        if(SExist) {
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

private:
    int i;
    Eigen::Matrix3d SN;
    Eigen::Matrix3d Q;
    double R;
    Eigen::Matrix3d A;
    Eigen::Matrix3d S;
    Eigen::Matrix<double, 3, 1> B;
    Eigen::Matrix<double, 1, 3> K;
    Eigen::Matrix<double, 3, 1> X;
    bool SExist = true;
    Eigen::VectorXd inclineX = Eigen::VectorXd(2);
    double estPlane = 0.0;
    double rodAcc = 0.0;

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
    void calEstPlane();
    bool IsTorqueZero();
};


#endif //RAISIM_MIPILQRCONTROLLER_H
