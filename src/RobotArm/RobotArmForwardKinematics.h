//
// Created by jy on 22. 7. 6.
//

#ifndef RAISIM_ROBOTARMFORWARDKINEMATICS_H
#define RAISIM_ROBOTARMFORWARDKINEMATICS_H
#include "Eigen/Eigen"
#include "src/RobotArm/RemoveMatrix.h"

class RobotArmForwardKinematics {

public:

    Eigen::MatrixXd forwardKinematics(Eigen::MatrixXd);
    RemoveMatrix removeMatrix;

private:

    enum Axes{
        ROLL,
        PITCH,
        YAW
    };

    Eigen::MatrixXd rotationMatrix(int axes, double radian);
    Eigen::MatrixXd translationMatrix(double x, double y, double z);
    Eigen::MatrixXd mT01 = Eigen::MatrixXd(4,4);
    Eigen::MatrixXd mT12 = Eigen::MatrixXd(4,4);
    Eigen::MatrixXd mT23 = Eigen::MatrixXd(4,4);
    Eigen::MatrixXd mT34 = Eigen::MatrixXd(4,4);
    Eigen::MatrixXd mT45 = Eigen::MatrixXd(4,4);
    Eigen::MatrixXd mT56 = Eigen::MatrixXd(4,4);
    Eigen::MatrixXd mT67 = Eigen::MatrixXd(4,4);

};


#endif //RAISIM_ROBOTARMFORWARDKINEMATICS_H
