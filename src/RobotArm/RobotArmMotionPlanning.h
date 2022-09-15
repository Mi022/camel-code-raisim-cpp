//
// Created by jy on 22. 7. 6.
//

#ifndef RAISIM_ROBOTARMMOTIONPLANNING_H
#define RAISIM_ROBOTARMMOTIONPLANNING_H
#include "Eigen/Eigen"
#include "iostream"
#include <time.h>
#include "src/RobotArm/RobotArmCollisionChecker.h"
#include "src/RobotArm/DistanceCalculator.h"
#include "src/RobotArm/RobotArmForwardKinematics.h"
#include "src/RobotArm/RemoveMatrix.h"
#include "include/TrajectoryGenerator/RobotArmTrajectoryGenerator.h"

class RobotArmMotionPlanning {
public:
    RobotArmMotionPlanning(RobotArmCollisionChecker* collisionChecker, RobotArmTrajectoryGenerator* trajectoryGenerator){
        startJoint << 0.0, 2.79, -1.57, 0.0, 1.98, 0.0;
        goalJoint << 3.14, 2.0, -2.6, 2.0, 3.14, 0.0;
        startJoint = (180/3.141592)*startJoint;
        goalJoint = (180/3.141592)*goalJoint;
        this->collisionChecker = collisionChecker;
        this->trajectoryGenerator = trajectoryGenerator;
    }
    Eigen::MatrixXd armPose = Eigen::MatrixXd(1,6);
    int len ;
    Eigen::MatrixXd pare = Eigen::MatrixXd::Zero(1,3);
    Eigen::MatrixXd pareAdd = Eigen::MatrixXd::Zero(1,3);
    Eigen::MatrixXd childTree ;
    std::vector<int> findTree;
    Eigen::MatrixXd wayPoints;

    void generatePoint();
    void makeTree();
    void dijkstra();

    RobotArmForwardKinematics forwardKinematics;
    RobotArmCollisionChecker* collisionChecker;
    DistanceCalculator distance;
    RemoveMatrix removeMatrix;
    RobotArmTrajectoryGenerator* trajectoryGenerator;

private:
    Eigen::VectorXd mObstacleRadius = Eigen::VectorXd(2);
    Eigen::MatrixXd mObstacleCenter = Eigen::MatrixXd(2,3);
    Eigen::VectorXd startJoint = Eigen::VectorXd(6);
    Eigen::VectorXd goalJoint = Eigen::VectorXd(6);
    time_t treeStart,treeEnd,searchStart,searchEnd;

};


#endif //RAISIM_ROBOTARMMOTIONPLANNING_H
