//
// Created by user on 22. 6. 17.
//

#ifndef RAISIM_MIPMPCCONTROLLER_H
#define RAISIM_MIPMPCCONTROLLER_H

#include "include/CAMEL/Controller.h"

#define DELTA 1e-6
#define STEP_SIZE 0.01
#define ITERATION 1000
#define TERMINATE_CONDITION 1e-5

using namespace Eigen;

class MIPMPCController : public Controller {
public:
    MIPMPCController(Robot *robot, double dT) : Controller(robot) {
        mU.setZero();
        mTorque.setZero();
        this->mDT = dT;
        setQGain(1.0, 0.001, 0.001);
        setRGain(0.05);
    }

    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;

    const VectorXd &getTorque() const;
    void setTorqueLimit(double torqueLimit);

private:
    raisim::VecDyn mPosition = raisim::VecDyn(2);
    raisim::VecDyn mVelocity = raisim::VecDyn(2);
    VectorXd mTorque = VectorXd(2);
    double mTorqueLimit = 3.5;
    double mDesirePosition;
    Matrix3d mQ;
    double mR;
    int mMPCHorizen = 5;
    VectorXd mU = VectorXd(mMPCHorizen);
    VectorXd mX = VectorXd(3);
    VectorXd mX_temp = VectorXd(3);
    VectorXd mX_bar = VectorXd(3);
    VectorXd mDJ = VectorXd(mMPCHorizen);
    double mDT;

    void setQGain(double Q11, double Q22, double Q33);
    void setRGain(double R);
    void updateU();
    void computeDJ();
    double computeJ(VectorXd U);
    void stateSpaceEquation(double u);
    bool IsBreak(int i);
};


#endif //RAISIM_MIPMPCCONTROLLER_H
