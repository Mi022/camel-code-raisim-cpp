//
// Created by user on 22. 6. 11.
// this controller is for balancing
// desired states (position1, velocity1, velocity2) are zeros
//

#include "MIPLQRController.h"
using namespace std;

void MIPLQRController::setMatrix() {
    A <<    1.00165, 0.005, 0.0,
            0.6606, 1.0, 0.0,
            -0.6606, 0.0, 1.0;
    B <<    0.00718,
            -2.872,
            24.442;
}
void MIPLQRController::setSNGain(double SN11, double SN22, double SN33) {
    SN<<    SN11, 0.0, 0.0,
            0.0, SN22, 0.0,
            0.0, 0.0, SN33;
}

void MIPLQRController::setQGain(double Q11, double Q22, double Q33) {
    Q <<    Q11, 0.0, 0.0,
            0.0, Q22, 0.0,
            0.0, 0.0, Q33;
}

void MIPLQRController::setRGain(double R) {
    this -> R = R;
}

void MIPLQRController::findS() {
    int iteration = 0;
    Eigen::Matrix3d snext = SN;
    double temp = B.transpose() * snext * B + R;
    S = A.transpose() * (snext - snext * B / temp * B.transpose() * snext) * A + Q;
    while(!IsSEnough(S, snext)){
         iteration++;
         cout<<"check: "<<iteration<<endl;
         if(iteration > 10000)
         {
             SExist = false;
             break;
         }
         snext = S;
         S = A.transpose() * (snext - snext * B / (B.transpose() * snext * B + R) * B.transpose() * snext) * A + Q;
         cout<<"check S :\n"<<S<<endl;
    }
}

bool MIPLQRController::IsSEnough(Eigen::Matrix3d S, Eigen::Matrix3d Snext) {
    bool tempJudge = true;
    for(int r = 0; r<3; r++){
        for(int c = 0; c<3; c++){
            tempJudge = tempJudge && abs(S(r,c) - Snext(r, c)) < 0.001;
        }
    }
    if(tempJudge){
        return true;
    }
    else{
        return false;
    }
}
void MIPLQRController::findK() {
    double temp = B.transpose()*S*B + R;
    K = B.transpose()*S*A/temp;
    cout<<"K : "<<K<<endl;
}

void MIPLQRController::generateExternalForce() {
    raisim::Vec<3> externalForce;
    raisim::Vec<3> forcePosition;

    externalForce.setZero();
    forcePosition.setZero();

    forcePosition[0] = 0.0;
    forcePosition[1] = 0.0;
    forcePosition[2] = 0.08;

    externalForce[0] = 0.0;
    externalForce[1] = 9.6;
    externalForce[2] = 0.0;

    if(i%400 == 0 || i == 0){
        std::cout<<"force"<<std::endl;
        getRobot()->robot->setExternalForce(1, forcePosition, externalForce);
    }
    i++;
}

void MIPLQRController::addNoise() {
    std::random_device rd;

    // random_device 를 통해 난수 생성 엔진을 초기화 한다.
    std::mt19937 gen(rd());

    // 0 부터 200 까지 균등하게 나타나는 난수열을 생성하기 위해 균등 분포 정의.
    std::uniform_int_distribution<int> dis(0, 200);
    double noisePosition = (double(dis(gen)) / 100.0 - 1.0) * 0.001; //0.15
    double noiseVelocity = (double(dis(gen)) / 100.0 - 1.0) * 0.01; //1.6
    double noiseMotorVelocity = (double(dis(gen)) / 100.0 - 1.0) * 0.001; //35.9

    position[0] += noisePosition;
    velocity[0] += noiseVelocity;
    velocity[1] += noiseMotorVelocity;
}

void MIPLQRController::doControl() {
    updateState();
//    generateExternalForce();
    if(SExist)
    {
        computeControlInput();
        setControlInput();
    }
}

void MIPLQRController::updateState() {
//    for ideal
    position = getRobot()->robot->getGeneralizedCoordinate();
    velocity = getRobot()->robot->getGeneralizedVelocity();

//    for adding noise
//    addNoise();

    X[0] = position[0];
    X[1] = velocity[0];
    X[2] = velocity[1];
}

void MIPLQRController::computeControlInput() {
//    torque[1] = - (K[0]*X[0] + K[1]*X[1] + K[2]*X[2]);
    torque[1] = -K*X;
    if(torque[1] > torqueLimit)
    {
        torque[1] = torqueLimit;
    }
    else if(torque[1] < -torqueLimit)
    {
        torque[1] = -torqueLimit;
    }
}

void MIPLQRController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(torque);
}

void MIPLQRController::setTrajectory() {

}