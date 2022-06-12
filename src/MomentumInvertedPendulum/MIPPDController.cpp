//
// Created by user on 22. 6. 10.
//

#include "MIPPDController.h"

void MIPPDController::setPDGain(double PGain, double DGain) {
    this->PGain = PGain;
    this->DGain = DGain;
}

void MIPPDController::generateExternalForce() {
    raisim::Vec<3> externalForce;
    raisim::Vec<3> forcePosition;

    externalForce.setZero();
    forcePosition.setZero();

    forcePosition[0] = 0.0;
    forcePosition[1] = 0.0;
    forcePosition[2] = 0.08;

    externalForce[0] = 0.0;
    externalForce[1] = 5.0;
    externalForce[2] = 0.0;

    if(i%400 == 0 || i == 0){
        std::cout<<"force"<<std::endl;
        getRobot()->robot->setExternalForce(1, forcePosition, externalForce);
    }
    i++;
}

void MIPPDController::addNoise() {
    std::random_device rd;

    // random_device 를 통해 난수 생성 엔진을 초기화 한다.
    std::mt19937 gen(rd());

    // 0 부터 99 까지 균등하게 나타나는 난수열을 생성하기 위해 균등 분포 정의.
    std::uniform_int_distribution<int> dis(0, 200);
    double noisePosition = (double(dis(gen)) / 100.0 - 1.0) * 0.001;
    double noiseVelocity = (double(dis(gen)) / 100.0 - 1.0) * 0.01;

    position[0] += noisePosition;
    velocity[0] += noiseVelocity;

}

void MIPPDController::doControl() {
    updateState();
    generateExternalForce();
    computeControlInput();
    setControlInput();
}

void MIPPDController::setTrajectory() {
    desiredJointPosition[0] = 0.0;
    desiredJointVelocity[0] = 0.0;
}

void MIPPDController::updateState() {
    position = getRobot()->robot->getGeneralizedCoordinate();
    velocity = getRobot()->robot->getGeneralizedVelocity();

    addNoise();
}

void MIPPDController::computeControlInput() {
    positionError[0] = desiredJointPosition[0] - position[0];
    velocityError[0] = desiredJointVelocity[0] - velocity[0];

    torque[1] = -PGain*positionError[0] - DGain*velocityError[0];

    if(torque[1] > torqueLimit)
    {
        torque[1] = torqueLimit;
    }
    else if(torque[1] < -torqueLimit)
    {
        torque[1] = -torqueLimit;
    }
}

void MIPPDController::setControlInput() {
 getRobot()->robot->setGeneralizedForce(torque);
}