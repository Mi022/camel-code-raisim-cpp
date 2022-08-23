//
// Created by jy on 22. 7. 6.
//

#include "RobotArmForwardKinematics.h"
#include "iostream"
#include "cmath"

Eigen::MatrixXd RobotArmForwardKinematics::rotationMatrix(char axes, double radian) {
    Eigen::MatrixXd(4,4) matrix;
    if(axes == 'r'){
        matrix << ;
    }
    else if(axes == 'p'){
        matrix << ;
    }
    else if(axes == 'y'){
        matrix << ;
    }
}

Eigen::MatrixXd RobotArmForwardKinematics::translationMatrix(double x, double y, double z) {


}




Eigen::MatrixXd RobotArmForwardKinematics::forwardKinematics(Eigen::MatrixXd joint) {
    float pi = 3.141592;
    joint = joint*(pi/180);
    Eigen::VectorXd linkLength = Eigen::VectorXd::Random(8);
    Eigen::MatrixXd linkPoint = Eigen::MatrixXd::Random(6,3);
    linkLength << 1,2,3,4,5,6,7,8;
    linkPoint << 3,4,linkLength(3)*4,
            linkLength(3)* sin(joint(0)),2,3,
            4,5,6,
            1,0,5,
            1,2,3,
            4,5,6;

    return linkPoint;
}

Eigen::MatrixXd RobotArmForwardKinematics::rotRoll(float joint) {
    Eigen::MatrixXd rotR = Eigen::MatrixXd(4,4);

    return rotR;
}

Eigen::MatrixXd RobotArmForwardKinematics::rotPitch(float joint) {
    Eigen::MatrixXd rotP = Eigen::MatrixXd(4,4);

    return rotP;
}

Eigen::MatrixXd RobotArmForwardKinematics::rotYaw(float joint) {
    Eigen::MatrixXd rotY = Eigen::MatrixXd(4,4);

    return rotY;
}

Eigen::MatrixXd RobotArmForwardKinematics::translation(float xD, float yD, float zD) {


}