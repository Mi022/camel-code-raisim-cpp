//
// Created by jy on 22. 7. 6.
//

#ifndef RAISIM_ROBOTARMFORWARDKINEMATICS_H
#define RAISIM_ROBOTARMFORWARDKINEMATICS_H
#include "Eigen/Eigen"

class RobotArmForwardKinematics {

public:

    Eigen::MatrixXd forwardKinematics(Eigen::MatrixXd);

private:
    Eigen::MatrixXd rotationMatrix(char axes, double radian);
    Eigen::MatrixXd translationMatrix(double x, double y, double z);
};


#endif //RAISIM_ROBOTARMFORWARDKINEMATICS_H
