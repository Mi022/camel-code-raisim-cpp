
#include "TwoLeggedPDController.h"

void TwoLeggedPDController::setPDGain(Eigen::Vector4d PGain, Eigen::Vector4d DGain) {
    this->PGain = PGain;
    this->DGain = DGain;
}

void TwoLeggedPDController::doControl() {
    updateState();
    setTrajectory();
    IKsolve();
    computeControlInput();
    setControlInput();
}

void TwoLeggedPDController::setTrajectory() {
//    desiredPosition = 0;
//    desiredVelocity = 0;
}

void TwoLeggedPDController::updateState() {
    position = getRobot()->getQ();
    velocity = getRobot()->getQD();
    mTheta[0] = getRobot()->getQ()[7] - 90*deg2rad;
    mTheta[1] = getRobot()->getQ()[8];
    mTheta[2] = getRobot()->getQ()[9] - 90*deg2rad;
    mTheta[3] = getRobot()->getQ()[10];
}

void TwoLeggedPDController::computeControlInput() {
    for (int i = 6; i < 10; i++) {
        if(i%2){//torque7, 9
            torque[i] = -mass*gravity*l*sin(mTheta[i-7]+mTheta[i-6])/2;
            std::cout<<mass*gravity*l<<std::endl;
        }else{//torque6, 8
            torque[i] = -mass*gravity*l*(sin(mTheta[i-6]+mTheta[i-5])+sin(mTheta[i-6]))/2;
        }
//        if(torque[i] > torqueLimit)
//        {
//            torque[i] = torqueLimit;
//        }
//        else if(torque[i] < -torqueLimit)
//        {
//            torque[i] = -torqueLimit;
//        }
    }
//    torque.setZero();
//    torque[2] = 100;
}


void TwoLeggedPDController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(torque);
}

void TwoLeggedPDController::IKsolve() {
//    desiredJointPosition[0] = acos(desiredPosition / 0.46);
//    desiredJointPosition[1] = -2*desiredJointPosition[0];
//    desiredJointVelocity.setZero();
}

void TwoLeggedPDController::mSetDesired() {
//    desiredPosition = getRobot()->getQ();
//    desiredVelocity = getRobot()->getQD();
    desiredPosition.setZero();
    desiredVelocity.setZero();
    //base position (x,y,z)
    desiredPosition[0] = 0.0;  //prismatic joint
    desiredPosition[1] = 0.0;
    desiredPosition[2] = 0.5;

    //rotation[quaternion]
    desiredPosition[3] = 0.0;
    desiredPosition[4] = 0.7071068;
    desiredPosition[5] = 0.0;
    desiredPosition[6] = 0.7071068;

    //BR-hip
    desiredPosition[7] = 1.5707963;

    //BL-hip
    desiredPosition[9] = 1.5707963;
}

const double TwoLeggedPDController::mass = 5.0 + 2*(0.8 + 0.2*2 + 0.06);
const double TwoLeggedPDController::gravity = 9.8;
const double TwoLeggedPDController::l = 0.2;
const double TwoLeggedPDController::deg2rad = 3.141592/180;