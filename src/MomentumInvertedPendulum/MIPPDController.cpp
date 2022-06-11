//
// Created by user on 22. 6. 10.
//

#include "MIPPDController.h"

void MIPPDController::setPDGain(double PGain, double DGain) {
    this->PGain = PGain;
    this->DGain = DGain;
}

void MIPPDController::doControl() {
    updateState();
    computeControlInput();
    setControlInput();
}

void MIPPDController::setTrajectory() {
    desiredJointPosition[0] = 0.0;
    desiredJointVelocity[0] = 0.0;
}

void MIPPDController::updateState() {
//    position = getRobot()->robot->getGeneralizedCoordinate();
//    velocity = getRobot()->robot->getGeneralizedVelocity();
    std::random_device rd;

    // random_device 를 통해 난수 생성 엔진을 초기화 한다.
    std::mt19937 gen(rd());

    // 0 부터 99 까지 균등하게 나타나는 난수열을 생성하기 위해 균등 분포 정의.
    std::uniform_int_distribution<int> dis(0, 99);
    double noisePosition = (double(dis(gen)) / 100.0 - 0.5) * 0.001;
    double noiseVelocity = (double(dis(gen)) / 100.0 - 0.5) * 0.01;
    position = getRobot()->robot->getGeneralizedCoordinate();
    velocity = getRobot()->robot->getGeneralizedVelocity();
    position[0] += noisePosition;
    velocity[0] += noiseVelocity;

    if(i%400 == 0 || i == 0){
        std::cout<<"force"<<std::endl;
        getRobot()->robot->setExternalForce(1, forcePosition, externalForce);
    }
    i++;
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