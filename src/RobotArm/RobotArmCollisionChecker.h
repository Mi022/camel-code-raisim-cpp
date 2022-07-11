//
// Created by jy on 22. 7. 6.
//

#ifndef RAISIM_ROBOTARMCOLLISIONCHECKER_H
#define RAISIM_ROBOTARMCOLLISIONCHECKER_H
#include "Eigen/Eigen"
#include "src/RobotArm/DistanceCalculator.h"
#include "src/RobotArm/LineInterpolation.h"
#include "src/RobotArm/RobotArmForwardKinematics.h"

class RobotArmCollisionChecker {

public:
    void setObstacle(Eigen::VectorXd, Eigen::MatrixXd);
    bool collisionCircle(Eigen::MatrixXd center,double radius, Eigen::MatrixXd point);
    bool obstacleChecker(Eigen::MatrixXd point);
    bool lineChecker(Eigen::MatrixXd point1,Eigen::MatrixXd point2);
    bool jointChecker(Eigen::MatrixXd joint);

    DistanceCalculator distanceCalculator;
    LineInterpolation interpolation;
    RobotArmForwardKinematics forwardKinematics;
private:
    Eigen::VectorXd mObstacleRadius = Eigen::VectorXd(2);
    Eigen::MatrixXd mObstacleCenter = Eigen::MatrixXd(2,3);

};


#endif //RAISIM_ROBOTARMCOLLISIONCHECKER_H
