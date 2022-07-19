//
// Created by jaehoon on 22. 5. 25.
//

#ifndef RAISIM_SINGLELEGGEDROBOTOPERATION_H
#define RAISIM_SINGLELEGGEDROBOTOPERATION_H

#include "include/CAMEL/OperationRobot.h"
#include "SingleLegCAN.h"
//TODO: update mLumpedMass

class SingleLeggedRobotOperation : public OperationRobot {
public:
    SingleLeggedRobotOperation(raisim::World *world, std::string urdfPath, std::string name, SingleLegCAN *can, double dT) : OperationRobot(
            world, urdfPath, name) {
        mCan = can;
        mDT = dT;
        initialize();
    }

    void initialize() override;
    void visualize();
    void setTorque(Eigen::VectorXd torque);
    Eigen::VectorXd getQ();
    Eigen::VectorXd getQD();
    double getPlot1();
    double getPlot2();

private:
    SingleLegCAN *mCan;
    Eigen::VectorXd mJointPosition = Eigen::VectorXd(3);
    Eigen::VectorXd mJointPosition_past = Eigen::VectorXd(3);
    Eigen::VectorXd mJointVelocity = Eigen::VectorXd(3);

    double mHipOffset = 1.38045057;
    double mKneeOffset = -2.649026222;
    double mLumpedMass = 2.660;
    double mDT;
    int mMotorKneeID = 0x141;
    int mMotorHipID = 0x143;
};


#endif //RAISIM_SINGLELEGGEDROBOTOPERATION_H
