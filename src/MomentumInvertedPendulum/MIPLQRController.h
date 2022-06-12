//
// Created by user on 22. 6. 11.
//

#ifndef RAISIM_MIPLQRCONTROLLER_H
#define RAISIM_MIPLQRCONTROLLER_H

#include "include/CAMEL/Controller.h"
#include <Eigen/Core>

class MIPLQRController : public Controller {
public:
    Eigen::VectorXd torque = Eigen::VectorXd(2);
    raisim::VecDyn position = raisim::VecDyn(2);
    raisim::VecDyn velocity = raisim::VecDyn(2);



//    raisim::Vec<3> externalForce;
//    raisim::Vec<3> forcePosition;

    double torqueLimit = 3.5;

    MIPLQRController(Robot *robot) : Controller(robot) {
        i = 0;
        setMatrix();
        setSNGain(100.0, 100.0, 100.0);
        setQGain(10.0, 100.0, 1.0);
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
