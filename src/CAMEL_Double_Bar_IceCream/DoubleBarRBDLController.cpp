//
// Created by hwayoung on 22. 9. 22.
//

#include "DoubleBarRBDLController.h"

void DoubleBarRBDLController::doControl() {
    setTrajectory();
    updateState();
//    computeControlInput();
    setControlInput();
}

void DoubleBarRBDLController::setTrajectory() {
    desiredVelocity.setZero();
    desiredAccerelation = (desiredVelocity - velocity)/0.005;
}

void DoubleBarRBDLController::updateState() {
    position = getRobot()->getQ().e();
    velocity = getRobot()->getQD().e();
}

void DoubleBarRBDLController::computeControlInput() {
    std::cout<<"Tau_Before : "<<std::endl<<torque<<std::endl;
    std::cout << "desired:" << std::endl<< "Q:" << std::endl<< position << std::endl<< "QDot:" << std::endl<< velocity << std::endl<< "QDDot:" << std::endl<< desiredAccerelation << std::endl;
    RigidBodyDynamics::InverseDynamics(*model, position, velocity, desiredAccerelation, torque);
    std::cout<<"Tau_After : "<<std::endl<<torque<<std::endl;
}

void DoubleBarRBDLController::setControlInput() {
//    torque[0] = 0;
//    torque<<0, 1.15859, -0.0116645;
    getRobot()->robot->setGeneralizedForce(torque);
    std::cout<<"Tau : "<<std::endl<<torque<<std::endl;
}

void DoubleBarRBDLController::getModelFromURDF() {
    model = new RigidBodyDynamics::Model();
    std::string rbdlSourcePath = URDF_RSC_DIR;
    std::string modelFile = rbdlSourcePath;
    modelFile.append("/camel_double_bar_IceCream.urdf");
    std::cout<<"hi"<<std::endl;
    bool modelLoaded = RigidBodyDynamics::Addons::URDFReadFromFile(modelFile.c_str(), model, false);
    std::cout<<"check: "<<modelLoaded<<std::endl;
}