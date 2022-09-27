//
// Created by jy on 22. 7. 6.
//

#include "RobotArmForwardKinematics.h"
#include "iostream"
#include "cmath"

Eigen::MatrixXd RobotArmForwardKinematics::rotationMatrix(int axes, double radian)
{
    Eigen::MatrixXd matrix(4, 4);
    if (axes == ROLL)
    {
        matrix << 1, 0, 0, 0, 0, cos(radian), -sin(radian), 0, 0, sin(radian), cos(radian), 0, 0, 0, 0, 1;
    }
    else if (axes == PITCH)
    {
        matrix << cos(radian), 0, sin(radian), 0, 0, 1, 0, 0, -sin(radian), 0, cos(radian), 0, 0, 0, 0, 1;
    }
    else if (axes == YAW)
    {
        matrix << cos(radian), -sin(radian), 0, 0, sin(radian), cos(radian), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    }

    return matrix;
}

Eigen::MatrixXd RobotArmForwardKinematics::translationMatrix(double x, double y, double z)
{
    Eigen::MatrixXd matrix(4, 4);
    matrix << 1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1;

    return matrix;
}


Eigen::MatrixXd RobotArmForwardKinematics::forwardKinematics(Eigen::MatrixXd joint)
{
    float pi = 3.14159265359;
    joint = (pi / 180) * joint;
    Eigen::MatrixXd linkPoint(7, 4);
    Eigen::VectorXd linkLength(8);
    Eigen::VectorXd start(4);
    start << 0, 0, 0, 1;

    mT01 = translationMatrix(0, 0, 0.15675) * rotationMatrix(PITCH, pi) * rotationMatrix(YAW, joint(0));
    mT12 = translationMatrix(0, 0.0016, -0.11875) * rotationMatrix(YAW, pi) * rotationMatrix(ROLL, -pi / 2) * rotationMatrix(YAW, joint(1));
    mT23 = translationMatrix(0, -0.410, 0) * rotationMatrix(PITCH, pi) * rotationMatrix(YAW, joint(2));
    mT34 = translationMatrix(0, 0.2073, -0.0114) * rotationMatrix(YAW, pi) * rotationMatrix(ROLL, -pi / 2) * rotationMatrix(YAW, joint(3));
    mT45 = translationMatrix(0, 0, -0.10375) * rotationMatrix(YAW, pi) * rotationMatrix(ROLL, pi / 2) * rotationMatrix(YAW, joint(4));
    mT56 = translationMatrix(0, 0.10375, 0) * rotationMatrix(YAW, pi) * rotationMatrix(ROLL, -pi / 2) * rotationMatrix(YAW, joint(5));
    mT67 = translationMatrix(0, 0, -0.1600) * rotationMatrix(ROLL, pi);

    linkPoint.row(0) = mT01.col(3);
    linkPoint.row(1) = (mT01 * mT12).col(3);
    linkPoint.row(2) = (mT01 * mT12 * mT23).col(3);
    linkPoint.row(3) = (mT01 * mT12 * mT23 * mT34).col(3);
    linkPoint.row(4) = (mT01 * mT12 * mT23 * mT34 * mT45).col(3);
    linkPoint.row(5) = (mT01 * mT12 * mT23 * mT34 * mT45 * mT56).col(3);
    linkPoint.row(6) = (mT01 * mT12 * mT23 * mT34 * mT45 * mT56 * mT67).col(3);

    linkPoint = removeMatrix.removeColumn(linkPoint, linkPoint.cols());

    return linkPoint;
}