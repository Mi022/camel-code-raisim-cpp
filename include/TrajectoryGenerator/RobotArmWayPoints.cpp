//
// Created by jy on 22. 7. 5.
//

#include "RobotArmWayPoints.h"

double RobotArmWayPoints::setObstalce(double *obstacleRadius, double (*obstacleCenter)[3]) {
    mObstacleRadius[0] = obstacleRadius[0];
    mObstacleRadius[1] = obstacleRadius[1];
    mObstacleCenter[0][0] = obstacleCenter[0][0];
    *mObstacleRadius = *obstacleRadius;

}