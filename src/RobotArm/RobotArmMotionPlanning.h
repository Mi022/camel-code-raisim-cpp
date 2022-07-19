//
// Created by jy on 22. 7. 6.
//

#ifndef RAISIM_ROBOTARMMOTIONPLANNING_H
#define RAISIM_ROBOTARMMOTIONPLANNING_H
#include "Eigen/Eigen"
#include "src/RobotArm/RobotArmCollisionChecker.h"
#include "src/RobotArm/DistanceCalculator.h"
#include "src/RobotArm/RobotArmForwardKinematics.h"
#include "src/RobotArm/RemoveMatrix.h"
#include "iostream"


class RobotArmMotionPlanning {
public:
    RobotArmMotionPlanning(){
        startJoint << 0,0,0,0,0,0;
        goalJoint << 30,-30,-90,90,60,0;
    }
    Eigen::MatrixXd armPose = Eigen::MatrixXd(1,6);
    int len ;
    std::vector<int> findTree;
    Eigen::MatrixXd pare = Eigen::MatrixXd::Random(1,3);
    Eigen::MatrixXd pareAdd = Eigen::MatrixXd::Zero(1,3);
    Eigen::MatrixXd childTree ;

    void setObstacle(Eigen::VectorXd, Eigen::MatrixXd);
    void generatePoint();
    void makeTree();
    void dijkstra();

    RobotArmForwardKinematics forwardKinematics;
    RobotArmCollisionChecker collisionChecker;
    DistanceCalculator distance;
    RemoveMatrix removeMatrix;

//    Eigen::VectorXd getObstacleRadius(){return mObstacleRadius;}
//    Eigen::MatrixXd getObstacleCenter(){return mObstacleCenter;}

private:
    Eigen::VectorXd mObstacleRadius = Eigen::VectorXd(2);
    Eigen::MatrixXd mObstacleCenter = Eigen::MatrixXd(2,3);
    Eigen::VectorXd startJoint = Eigen::VectorXd(6);
    Eigen::VectorXd goalJoint = Eigen::VectorXd(6);


};


#endif //RAISIM_ROBOTARMMOTIONPLANNING_H
