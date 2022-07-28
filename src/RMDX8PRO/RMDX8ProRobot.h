//
// Created by jaehoon on 22. 4. 13..
//

#ifndef RAISIM_RMDX8PROROBOT_H
#define RAISIM_RMDX8PROROBOT_H

#include "include/CAMEL/Robot.h"
#include "include/CAN/CanMotorX8ProV2.h"

class RMDX8ProRobot : public Robot {
public:
    RMDX8ProRobot(raisim::World *world, std::string urdfPath, std::string name, CanMotorX8ProV2 *canMotorX8) : Robot(
            world, urdfPath, name) {
        mCanMotorX8 = canMotorX8;
        initialize();
    }

    void motorOn(int motorID);
    void motorOff(int motorID);
    void initialize() override;
    void setTorque(int motorID, int torque_int);
    double getQ(int motorID);
    double getQD(int motorID);

private:
    CanMotorX8ProV2 *mCanMotorX8;
};

void RMDX8ProRobot::initialize() {
    Eigen::VectorXd initialJointPosition(robot->getGeneralizedCoordinateDim());
    initialJointPosition.setZero();
    initialJointPosition[0] = getQ(0x141);
    robot->setGeneralizedCoordinate(initialJointPosition);
}

double RMDX8ProRobot::getQ(int motorID) {
    // from motor value
    mCanMotorX8->readMultiturnAngularPosition(motorID);
    return mCanMotorX8->getAngularPosition();
}

double RMDX8ProRobot::getQD(int motorID) {
    // from motor value
    mCanMotorX8->readMotorStatus2(motorID);
    return mCanMotorX8->getAngularVelocity();
}

void RMDX8ProRobot::setTorque(int motorID, int torque_int) {
    mCanMotorX8->setTorque(motorID, torque_int);
}

void RMDX8ProRobot::motorOn(int motorID) {
    mCanMotorX8->turnOnMotor(motorID);
}

void RMDX8ProRobot::motorOff(int motorID) {
    mCanMotorX8->turnOffMotor(motorID);
}

#endif //RAISIM_RMDX8PROROBOT_H
