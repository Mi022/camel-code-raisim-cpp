//
// Created by jy on 22. 6. 1.
//

#include "RobotArmTrajectoryGenerator.h"
#include "iostream"
#include "Eigen/Eigen"
#include "cmath"

void RobotArmTrajectoryGenerator::setWaypoints(Eigen::MatrixXd wayPoints) {
    mWaypoints = (3.141592/180)*wayPoints;
    pointNum = wayPoints.rows();

}

void RobotArmTrajectoryGenerator::updateTrajectory( double currentTime, double timeDuration) {

    mReferenceTime = currentTime;
    mTimeDuration = timeDuration;

}

void RobotArmTrajectoryGenerator::caculateCoefficient() {
    Eigen::MatrixXd xValue = Eigen::MatrixXd(2*pointNum , 2*pointNum);
    Eigen::MatrixXd yValue = Eigen::MatrixXd(2*pointNum,6);
    Eigen::MatrixXd xSVD;

    coefficient.conservativeResize(2*pointNum,6);
    float normalizeValue;
    normalizeValue = (1 + float (pointNum)) / 2;

    xValue.setZero();
    yValue.setZero();

    for(int rowNum = 0 ; rowNum < pointNum ; rowNum++){
        for(int poly =0 ; poly < 2*pointNum ; poly++){
            xValue(rowNum,poly) = pow(((rowNum+1)/normalizeValue)  ,poly);
        }
    }
    for(int rowNum = pointNum ; rowNum < 2*pointNum ; rowNum++){
        for(int poly = 1 ; poly < 2*pointNum ; poly++){
            xValue(rowNum,poly) = poly*xValue(rowNum-pointNum,poly-1);
        }
    }
    std::cout << "xValue" << std::endl;
    std::cout << xValue << std::endl;

    xSVD = xValue.transpose()*xValue;
    xSVD = xSVD.inverse();

    for(int i = 0 ; i < 6 ; i++){
        for(int rowNum = 0 ; rowNum < pointNum ; rowNum++){
            yValue(rowNum,i) = mWaypoints(rowNum,i);
        }
        coefficient.col(i) = xSVD*xValue.transpose()*yValue.col(i);
    }

    std::cout << "coefficient" << std::endl;
    std::cout << coefficient << std::endl;
}

std::vector<double> RobotArmTrajectoryGenerator::getPositionTrajectory(double currentTime) {
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    caculateCoefficient();
    std::vector<double> positionTrajectory(6);

    for (int i = 0; i < 6; i++) {
        for (int poly = 0; poly < 2*pointNum; poly++) {
            positionTrajectory[i] = positionTrajectory[i] + coefficient(poly,i)* pow(normalizedTime,poly);
        }
    }
    return positionTrajectory;
}

std::vector<double> RobotArmTrajectoryGenerator::getVelocityTrajectory(double currentTime) {
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    std::vector<double> velocityTrajectory(6);
    for (int i = 0; i < 6; i++) {
        for (int poly = 1; poly < 2*pointNum; poly++) {
            velocityTrajectory[i] = velocityTrajectory[i] + poly*coefficient(poly,i)*pow(normalizedTime,poly-1);
        }
    }
    return velocityTrajectory;
}

//double RobotArmTrajectoryGenerator::getAccelerationTrajectory(double currentTime) {
//    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
//    return (6.0 *mCoefficient(0,0) * normalizedTime + 2.0 * mCoefficient(1,0)) / pow(mTimeDuration, 2.0);
//}
