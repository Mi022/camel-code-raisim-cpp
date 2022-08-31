//
// Created by jy on 22. 7. 6.
//

#include "RobotArmCollisionChecker.h"
#include "iostream"

void RobotArmCollisionChecker::setObstacle(Eigen::VectorXd obstacleRadius, Eigen::MatrixXd obstacleCenter) {
    mObstacleCenter = obstacleCenter;
    mObstacleRadius = obstacleRadius;
}

bool RobotArmCollisionChecker::collisionCircle(Eigen::MatrixXd center,double radius, Eigen::MatrixXd point){
    if(distanceCalculator.distance(center,point) <= radius+5){
        return true;}
    else{
        return false;}
}

bool RobotArmCollisionChecker::obstacleChecker(Eigen::MatrixXd point){
    int collisionCount = 0;

    for(int i=0; i<mObstacleRadius.size() ;i++){
        if(collisionCircle(mObstacleCenter.row(i),mObstacleRadius(i),point)){
            collisionCount += 1 ;
        }
    }
    if(collisionCount > 0)
        return false;
    else
        return true;
}

bool RobotArmCollisionChecker::lineChecker(Eigen::MatrixXd point1,Eigen::MatrixXd point2){
    int splitSize = 3;
    Eigen::VectorXd xStep;
    Eigen::VectorXd yStep;
    Eigen::VectorXd zStep;
    xStep = interpolation.interpolation(point1(0),point2(0),splitSize);
    yStep = interpolation.interpolation(point1(1),point2(1),splitSize);
    zStep = interpolation.interpolation(point1(2),point2(2),splitSize);
//    std::cout << xStep(1) << yStep(2) << zStep(1) <<std::endl;
    Eigen::MatrixXd stepPoint = Eigen::MatrixXd(1,3);
    for(int i = 0; i<splitSize ; i++){
        stepPoint(0)=xStep[i];
        stepPoint(1)=yStep[i];
        stepPoint(2)=zStep[i];

        if(obstacleChecker(stepPoint)){
            continue;
        }
        else
            return false;
    }
    return true;
}

bool RobotArmCollisionChecker::jointChecker(Eigen::MatrixXd joint){
    Eigen::MatrixXd linkPoint = forwardKinematics.forwardKinematics(joint);
    for(int i =0; i < linkPoint.rows()-1 ; i++){
        if(lineChecker(linkPoint.row(i),linkPoint.row(i+1))){
            continue;
        }
        else
            return true;
    }
    return false;
}
