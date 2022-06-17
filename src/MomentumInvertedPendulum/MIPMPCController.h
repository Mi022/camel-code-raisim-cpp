//
// Created by user on 22. 6. 17.
//

#ifndef RAISIM_MIPMPCCONTROLLER_H
#define RAISIM_MIPMPCCONTROLLER_H

#include "include/CAMEL/Controller.h"

using namespace Eigen;
using namespace std;
class MIPMPCController : public Controller {
public:
    VectorXd torque = VectorXd(2);
    raisim::VecDyn position = raisim::VecDyn(2);
    raisim::VecDyn velocity = raisim::VecDyn(2);
    double desirePosition = 0.2;

    double torqueLimit = 3.5;

    MIPMPCController(Robot *robot, double dT) : Controller(robot) {
        U.setZero();
        torque.setZero();
        this->dT = dT;
        setQGain(1.5, 0.001, 0.001);
        setRGain(0.00001);

        delta = 1e-6;
        stepSize = 0.1;
        iteration = 10000;
        terminateCondition = 1e-7;
    }

    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;
    void setPDGain(double PGain, double DGain);

private:
    Matrix3d Q;
    double R;
    double stepSize;
    double delta;
    int MPCHorizen = 5;
    int iteration;
    double terminateCondition;

    VectorXd U = VectorXd(MPCHorizen);
    VectorXd X = VectorXd(3);
    VectorXd X_temp = VectorXd(3);
    VectorXd X_bar = VectorXd(3);
    VectorXd DJ = VectorXd(MPCHorizen);
    double dT;

    void setQGain(double Q11, double Q22, double Q33);
    void setRGain(double R);
    void updateU();
    void computeDJ();
    double computeJ(VectorXd U);
    void stateSpaceEquation(double u);
    bool IsBreak(int i);

};


#endif //RAISIM_MIPMPCCONTROLLER_H
