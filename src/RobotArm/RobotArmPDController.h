//
// Created by jy on 22. 4. 3..
//

#ifndef RAISIM_ROBOTARMPDCONTROLLER_H
#define RAISIM_ROBOTARMPDCONTROLLER_H

#include "include/CAMEL/Controller.h"
#include "include/TrajectoryGenerator/RobotArmTrajectoryGenerator.h"
#include <cmath>

class RobotArmPDController : public Controller {
public:
    Eigen::VectorXd torque = Eigen::VectorXd(6);
    raisim::VecDyn position = raisim::VecDyn(6);
    raisim::VecDyn velocity = raisim::VecDyn(6);
    Eigen::VectorXd positionError = Eigen::VectorXd(6);
    Eigen::VectorXd velocityError = Eigen::VectorXd(6);
    Eigen::VectorXd desiredPosition = Eigen::VectorXd(6);;
    Eigen::VectorXd desiredVelocity = Eigen::VectorXd(6);

    Eigen::VectorXd PGain = Eigen::VectorXd(6);
    Eigen::VectorXd DGain = Eigen::VectorXd(6);

    size_t TRFrameIndex = getRobot()->robot->getFrameIdxByName("top_roll");//변수
    size_t FBFrameIndex = getRobot()->robot->getFrameIdxByName("fixed_ball");
    raisim::Vec<3> P_position[2];


    RobotArmPDController(Robot *robot) : Controller(robot) {
        updateState();
        mTrajectoryGenerator.updateTrajectory( getRobot() -> getWorldTime(), 5.0);
        PGain << 40.0, 40.0, 40.0, 15.0, 15.0, 15.0;
        DGain << 1.0, 1.0, 1.0, 0.5, 0.5, 0.5;
        setPDGain(PGain, 5*DGain);
    }

    void doControl() override;
    void setTrajectory() override;
    void updateState() override;
    void computeControlInput() override;
    void setControlInput() override;
    void setPDGain(Eigen::VectorXd PGain, Eigen::VectorXd DGain);

    RobotArmTrajectoryGenerator* getTrajectoryGenerator(){return &mTrajectoryGenerator;}

private:
    RobotArmTrajectoryGenerator mTrajectoryGenerator;
    std::vector<double> positionTrajectory;
};



#endif //RAISIM_ROBOTARMPDCONTROLLER_H
