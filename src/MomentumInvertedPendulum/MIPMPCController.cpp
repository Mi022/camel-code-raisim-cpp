//
// Created by user on 22. 6. 17.
//

#include "MIPMPCController.h"

void MIPMPCController::setQGain(double Q11, double Q22, double Q33) {
    Q <<    Q11, 0.0, 0.0,
            0.0, Q22, 0.0,
            0.0, 0.0, Q33;
}

void MIPMPCController::setRGain(double R) {
    this -> R = R;
}

void MIPMPCController::updateU(){
    U = U - stepSize*DJ;
//    std::cout<<"test U: "<<U<<std::endl;
}

void MIPMPCController::computeDJ(){
    VectorXd Udelta = VectorXd(MPCHorizen);
    for(int i = 0; i<MPCHorizen; i++){
        Udelta = U;
        Udelta[i] = U[i] + delta;
        DJ[i] = (computeJ(Udelta) - computeJ(U))/delta;
//        std::cout<<" cost function: "<<computeJ(Udelta) <<"  "<< computeJ(U)<<std::endl;
    }
//    std::cout<<"test DJ : "<<DJ<<std::endl;
}

double MIPMPCController::computeJ(VectorXd U) {
    double J = 0;
    X_temp = X;
    for(int i = 0; i<MPCHorizen; i++){
        stateSpaceEquation(U[i]);
        X_bar = X_temp;
        X_bar[0] -= desirePosition;
        J += X_temp.transpose()*Q*X_temp + U[i]*R*U[i];
    }
//    std::cout<<"test J : "<<J<<std::endl;
    return J;
}

void MIPMPCController::stateSpaceEquation(double u) {
    double theta_0 = X_temp[0];
    X_temp[0] = theta_0 + dT*X_temp[1] + dT*dT/2*132.1654*sin(theta_0)  - dT*dT/0.0035*u;
    X_temp[1] = 132.1654*dT*sin(theta_0) + X_temp[1] -dT/0.001741*u;
    X_temp[2] = -132.1654*dT*sin(theta_0) + X_temp[2] + dT*4.8884e+03*u;

//    std::cout<<"test X next: "<<X<<std::endl;
}

bool MIPMPCController::IsBreak(int i){
    double RMS = pow((DJ.transpose()*DJ)(0)/MPCHorizen,0.5);
    if(RMS < terminateCondition){
        cout<<"DJ : "<<DJ<<endl;
        cout<<"RMS : "<<RMS<<endl;
        cout<<"Iteration : "<<i<<endl;
        cout<<"terminate Condition"<<endl;
        return false;
    }
    if(i> iteration){
        cout<<"RMS : "<<RMS<<endl;
        cout<<"maximun iteration"<<endl;
        return false;
    }
    return true;
}

void MIPMPCController::doControl() {
    updateState();
    computeControlInput();
//    std::cout<<"test"<<std::endl;
    setControlInput();
}

void MIPMPCController::setTrajectory() {

}

void MIPMPCController::updateState() {
    position = getRobot()->robot->getGeneralizedCoordinate();
    velocity = getRobot()->robot->getGeneralizedVelocity();

    X[0] = position[0];
    X[1] = velocity[0];
    X[2] = velocity[1];
}

void MIPMPCController::computeControlInput() {
    int i = 0;
    bool doing = true;
    while(doing)
    {
        computeDJ();
        updateU();
        doing = IsBreak(i);
        i++;
    }
    std::cout<<"torques : "<<U[0]<<std::endl;
    torque[1] = U[0];

    if(torque[1] > torqueLimit)
    {
        torque[1] = torqueLimit;
    }
    else if(torque[1] < -torqueLimit)
    {
        torque[1] = -torqueLimit;
    }
}

void MIPMPCController::setControlInput() {
    getRobot()->robot->setGeneralizedForce(torque);
    U.setZero();
}