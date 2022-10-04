//
// Created by jy on 22. 7. 6.
//

#include "RobotArmCollisionChecker.h"
#include "iostream"

void RobotArmCollisionChecker::setObstacle(Eigen::VectorXd obstacleRadius, Eigen::MatrixXd obstacleCenter)
{
    mObstacleCenter = obstacleCenter;
    mObstacleRadius = obstacleRadius;
}

bool RobotArmCollisionChecker::collisionCircle(Eigen::MatrixXd center, double radius, Eigen::MatrixXd point)
{
    if (distanceCalculator.distance(center, point) <= radius + 0.01)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * if collide obstacle return false , not collide obstacle return true
 * @param point
 * @return
 */
bool RobotArmCollisionChecker::obstacleChecker(Eigen::MatrixXd point)
{
    int collisionCount = 0;
    bool avoid;
    for (int i = 0; i < mObstacleRadius.size(); i++)
    {
        if (collisionCircle(mObstacleCenter.row(i), mObstacleRadius(i), point))
        {
            collisionCount += 1;
        }
    }
    if (collisionCount > 0)
    {
        avoid = false;
    }
    else
    {
        avoid = true;
    }

    return avoid;
}

/**
 * if exist obstacle between two points return false
 * @param point1
 * @param point2
 * @return
 */
bool RobotArmCollisionChecker::lineChecker(Eigen::MatrixXd point1, Eigen::MatrixXd point2)
{
    int splitSize = 50;
    Eigen::VectorXd xStep;
    Eigen::VectorXd yStep;
    Eigen::VectorXd zStep;
    Eigen::MatrixXd stepPoint = Eigen::MatrixXd(1, 3);

    xStep = interpolation.interpolation(point1(0), point2(0), splitSize);
    yStep = interpolation.interpolation(point1(1), point2(1), splitSize);
    zStep = interpolation.interpolation(point1(2), point2(2), splitSize);

    for (int i = 0; i < splitSize; i++)
    {
        stepPoint(0) = xStep[i];
        stepPoint(1) = yStep[i];
        stepPoint(2) = zStep[i];

        if (obstacleChecker(stepPoint))
        {
            continue;
        }
        else
        {
            return false;
        }
    }
    return true;
}

/**
 * if exist obstacle between each links return false
 * @param joint
 * @return
 */
bool RobotArmCollisionChecker::jointChecker(Eigen::MatrixXd joint)
{
    Eigen::MatrixXd linkPoint = forwardKinematics.forwardKinematics(joint);
    for (int i = 0; i < linkPoint.rows() - 1; i++)
    {
        if (lineChecker(linkPoint.row(i), linkPoint.row(i + 1)) and linkPoint(i, 2) > 0.00001 and linkPoint(i + 1, 2) > 0.00001)
        {
            continue;
        }
        else
        {
            return false;
        }
    }
    return true;
}
