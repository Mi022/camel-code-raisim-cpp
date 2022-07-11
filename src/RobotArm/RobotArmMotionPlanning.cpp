//
// Created by jy on 22. 7. 6.
//
#include "iostream"
#include "RobotArmMotionPlanning.h"

void RobotArmMotionPlanning::setObstacle(Eigen::VectorXd obstacleRadius, Eigen::MatrixXd obstacleCenter) {
    mObstacleCenter = obstacleCenter;
    mObstacleRadius = obstacleRadius;
//    std::cout << mObstacleCenter(0,2) << std::endl;
}

void RobotArmMotionPlanning::generatePoint() {
    int randNum = 200;
    armPose.row(0)=startJoint;
    int currentIndex = 0;
    for(int i=0 ; i<randNum ; i++){
        Eigen::MatrixXd joint = 180*Eigen::MatrixXd::Random(1,6);
        if(collisionChecker.jointChecker(joint)){
            currentIndex += 1;
            armPose.conservativeResize(armPose.rows()+1, armPose.cols());
            armPose.row(currentIndex)=joint;
        }
        else
            continue;
    }
    armPose.conservativeResize(armPose.rows()+1, armPose.cols());
    armPose.row(currentIndex+1) = goalJoint;
}

void RobotArmMotionPlanning::makeTree() {
    int k=100;
    int len = armPose.rows();
    std::cout << len << std::endl;
    Eigen::MatrixXd pare = Eigen::MatrixXd::Random(1,3);
    Eigen::MatrixXd pareAdd = Eigen::MatrixXd::Zero(1,3);
    Eigen::MatrixXd childTree = Eigen::MatrixXd::Zero(len,len);

    float lambda1;
    float lambda2;
    Eigen::MatrixXd currentPoint = Eigen::MatrixXd::Zero(6,3);
    Eigen::MatrixXd nextPoint = Eigen::MatrixXd::Zero(6,3);
    float currentDistance;
    int count = 0;

    currentPoint = forwardKinematics.forwardKinematics(pare);
    std::cout << currentPoint << std::endl;

    for(int i=0; i<len ; i++){
        int treeNum = 0;
        for(int j=0 ; j<len ; j++){
//            currentPoint = forwardKinematics.forwardKinematics(armPose.row(i));
//            nextPoint = forwardKinematics.forwardKinematics(armPose.row(j));
            currentDistance = distance.distance(armPose.row(i),armPose.row(j));
            if( 0 < currentDistance < k and collisionChecker.lineChecker(currentPoint.row(4),nextPoint.row(4))
                and collisionChecker.lineChecker(currentPoint.row(5),nextPoint.row(5))){
                pareAdd(0,0) = i;
                pareAdd(0,1) = j;
                pareAdd(0,2)=currentDistance;
                pare.row(count)=pareAdd;
                pare.conservativeResize(pare.rows()+1,pare.cols());
                count ++;
            }
        }
    }
    pare.conservativeResize(pare.rows()-1,pare.cols());
}