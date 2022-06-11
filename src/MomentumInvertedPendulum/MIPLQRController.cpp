//
// Created by user on 22. 6. 11.
// this controller is for balancing
// desired states (position1, velocity1, velocity2) are zeros
//

#include "MIPLQRController.h"
#include <Eigen/Core>
#include <Eigen/Dense>
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
    S = A.transpose() * (snext - snext * B / (B.transpose() * snext * B + R) * B.transpose() * snext) * A + Q;
    while(!IsSEnough(S, snext)){
         iteration++;
         cout<<"check: "<<iteration<<endl;
         if(iteration > 99)
         {
             SExist = false;
             break;
         }
         snext = S;
         cout<<"check\n"<<snext<<endl;
         S = A.transpose() * (snext - snext * B / (B.transpose() * snext * B + R) * B.transpose() * snext) * A + Q;
         cout<<"check\n"<<S<<endl;
    }
}

bool MIPLQRController::IsSEnough(Eigen::Matrix3d S, Eigen::Matrix3d Snext) {
    if(abs(S(0,0) - Snext(0, 0)) < 0.001){
        if(abs(S(1,1) - Snext(1, 1)) < 0.001){
            if(abs(S(2,2) - Snext(2, 2)) < 0.001){
                return true;
            }
            else{
                return false;
            }
        }
        else{
            return false;
        }
    }
    else{
        return false;
    }
}
void MIPLQRController::findK() {
    double temp = B.transpose()*S*B + R;
    K = B.transpose()*S*A/temp;
}

void MIPLQRController::doControl() {
    updateState();
    if(SExist)
    {
        computeControlInput();
        setControlInput();
    }
}

void MIPLQRController::updateState() {
    position = getRobot()->robot->getGeneralizedCoordinate();
    velocity = getRobot()->robot->getGeneralizedVelocity();
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