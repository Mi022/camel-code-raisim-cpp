//
// Created by jy on 22. 7. 6.
//

#include "RobotArmForwardKinematics.h"
#include "iostream"
#include "cmath"

Eigen::MatrixXd RobotArmForwardKinematics::rotationMatrix(int axes, double radian) {
    Eigen::MatrixXd matrix(4,4);
    if(axes == ROLL){
        matrix << 1, 0, 0, 0,
                0, cos(radian), -sin(radian), 0,
                0, sin(radian), cos(radian), 0,
                0, 0, 0, 1;
    }
    else if(axes == PITCH){
        matrix << cos(radian), 0, sin(radian), 0,
                0, 1, 0, 0,
                -sin(radian), 0, cos(radian), 0,
                0, 0, 0, 1;
    }
    else if(axes == YAW){
        matrix << cos(radian), -sin(radian), 0, 0,
                sin(radian), cos(radian), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
    }

    return matrix;
}

Eigen::MatrixXd RobotArmForwardKinematics::translationMatrix(double x, double y, double z) {
    Eigen::MatrixXd matrix(4,4);
    matrix << 1, 0, 0, x,
            0, 1, 0, y,
            0, 0, 1, z,
            0, 0, 0, 1;

    return matrix;
}


Eigen::MatrixXd RobotArmForwardKinematics::forwardKinematics(Eigen::MatrixXd joint) {
    float pi = 3.141592;
    joint = joint*(pi/180);
    Eigen::MatrixXd linkPoint(7,4);
    Eigen::VectorXd linkLength(8);
    Eigen::VectorXd start(4);
    linkLength << 0.15675,0.11875,0.0016,0.4100,0.2073,0.0014,0.10375,0.1600;
    start << 0,0,0,1;

    mT01 = translationMatrix(0,0,linkLength(0)) * rotationMatrix(ROLL,-pi);
    mT12 = rotationMatrix(YAW,joint(0)) * translationMatrix(0,linkLength(2),-linkLength(1)) * rotationMatrix(ROLL,pi/2);
    mT23 = rotationMatrix(YAW,joint(1)) * translationMatrix(0,-linkLength(3),0) * rotationMatrix(ROLL,pi);
    mT34 = rotationMatrix(YAW,joint(2)) * translationMatrix(0,-linkLength(4),linkLength(5)) * rotationMatrix(ROLL,pi/2);
    mT45 = rotationMatrix(YAW,joint(3)) * translationMatrix(0,0,-linkLength(6)) * rotationMatrix(ROLL,-pi/2);
    mT56 = rotationMatrix(YAW,joint(4)) * translationMatrix(0,linkLength(6),0) * rotationMatrix(ROLL,pi/2);
    mT67 = rotationMatrix(YAW,joint(5)) * translationMatrix(0,0,-linkLength(7)) * rotationMatrix(ROLL,pi) * rotationMatrix(ROLL,-pi);

    linkPoint.row(0) = mT01 * start;
    linkPoint.row(1) = mT01 * mT12 * start;
    linkPoint.row(2) = mT01 * mT12 * mT23 * start;
    linkPoint.row(3) = mT01 * mT12 * mT23 * mT34 * start;
    linkPoint.row(4) = mT01 * mT12 * mT23 * mT34 * mT45 * start;
    linkPoint.row(5) = mT01 * mT12 * mT23 * mT34 * mT45 * mT56 * start;
    linkPoint.row(6) = mT01 * mT12 * mT23 * mT34 * mT45 * mT56 * mT67 * start;

    linkPoint = removeMatrix.removeColumn(linkPoint,linkPoint.cols());
//    std::cout << linkPoint << std::endl;

    return linkPoint;
}