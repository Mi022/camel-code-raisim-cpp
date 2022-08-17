//
// Created by jaehoon on 22. 4. 3..
//

#ifndef RAISIM_SIMPLEPENDULUMPDCONTROLLER_H
#define RAISIM_SIMPLEPENDULUMPDCONTROLLER_H

#include <random>
#include "include/Filter/LPF.h"
#include "include/CAMEL/Controller.h"
#include "include/TrajectoryGenerator/QuinticTrajectoryGenerator.h"
#include "include/TrajectoryGenerator/SincurveTrajectoryGenerator.h"

class SingleLeggedPDController : public Controller {
public:
    Eigen::VectorXd torque = Eigen::VectorXd(3);
    raisim::VecDyn position = raisim::VecDyn(3);
    raisim::VecDyn velocity = raisim::VecDyn(3);
    Eigen::VectorXd positionError = Eigen::VectorXd(2);
    Eigen::VectorXd velocityError = Eigen::VectorXd(2);
    Eigen::VectorXd desiredJointPosition = Eigen::VectorXd(2);
    Eigen::VectorXd desiredJointVelocity = Eigen::VectorXd(2);

    double desiredPosition;
    double desiredVelocity;

    double PGain;
    double DGain;
    double torqueLimit = 13.0;

    SingleLeggedPDController(Robot *robot) : Controller(robot) {
        updateState();
        mTrajectoryGenerator.updateTrajectory(position[0], getRobot()->getWorldTime(), 0.05, 1.0);
        setPDGain(150.0, 2.5);
        torque[0] = 0.0;
        double cutoffFreq = 50.0;
        mLPF1.initialize(getRobot()->robotWorld->getTimeStep(), cutoffFreq);
        mLPF2.initialize(getRobot()->robotWorld->getTimeStep(), cutoffFreq);
    }

    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;
    void setPDGain(double PGain, double DGain);
    void IKsolve();
    raisim::VecDyn getNoisyQD();

private:
    SincurveTrajectoryGenerator mTrajectoryGenerator;
    raisim::VecDyn mNoisyQD = raisim::VecDyn(3);
    raisim::VecDyn mFilteredQD = raisim::VecDyn(3);
    std::default_random_engine mGenerator;
    std::normal_distribution<double> mDistribution = std::normal_distribution<double>(0.0, 0.09);
    LPF mLPF1;
    LPF mLPF2;
};


#endif //RAISIM_SIMPLEPENDULUMPDCONTROLLER_H
