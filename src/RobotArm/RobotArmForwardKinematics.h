//
// Created by jy on 22. 7. 6.
//

#ifndef RAISIM_ROBOTARMFORWARDKINEMATICS_H
#define RAISIM_ROBOTARMFORWARDKINEMATICS_H
#include "Eigen/Eigen"

class RobotArmForwardKinematics {

public:

    Eigen::MatrixXd forwardKinematics(Eigen::MatrixXd);

};


#endif //RAISIM_ROBOTARMFORWARDKINEMATICS_H
