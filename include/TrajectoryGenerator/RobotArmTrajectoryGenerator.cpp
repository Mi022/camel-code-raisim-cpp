//
// Created by jy on 22. 6. 1.
//

#include "RobotArmTrajectoryGenerator.h"
#include "iostream"
#include "Eigen/Eigen"
#include "math.h"

void RobotArmTrajectoryGenerator::setWaypoints(Eigen::MatrixXd wayPoints) {
    mWaypoints = wayPoints;
    pointNum = wayPoints.rows();

}

void RobotArmTrajectoryGenerator::updateTrajectory( double currentTime, double timeDuration) {

    mReferenceTime = currentTime;
    mTimeDuration = timeDuration;

}

void RobotArmTrajectoryGenerator::caculateCoefficient() {
    Eigen::MatrixXd xValue = Eigen::MatrixXd(2*pointNum , 2*pointNum);
    Eigen::MatrixXd yValue = Eigen::MatrixXd(2*pointNum,1);
    coefficient.conservativeResize(2*pointNum,6);

    xValue.setZero();
    for(int rowNum = 0 ; rowNum < pointNum ; rowNum++){
        for(int poly =0 ; poly < 2*pointNum ; poly++){
            xValue(rowNum,poly) = pow(rowNum+1,poly);
        }
    }
    for(int rowNum = pointNum ; rowNum < 2*pointNum ; rowNum++){
        xValue(rowNum,1) = 1;
        for(int poly = 2 ; poly < 2*pointNum ; poly++){
            xValue(rowNum,poly) = poly*(rowNum+1-pointNum);
        }
    }

    for(int i = 0 ; i < 6 ; i++){
        yValue.setZero();
        for(int rowNum = 0 ; rowNum < pointNum ; rowNum++){
            yValue(rowNum,0) = mWaypoints(rowNum,i);
        }
        xValue.inverse();
        coefficient.col(i) = xValue*yValue;
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

double RobotArmTrajectoryGenerator::getVelocityTrajectory(double currentTime) {
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
//    return (3.0 *mCoefficient(0,0) * pow(normalizedTime, 2.0) + 2.0 * mCoefficient(1,0) * normalizedTime + mCoefficient(2,0)) / mTimeDuration;
}


//double RobotArmTrajectoryGenerator::getAccelerationTrajectory(double currentTime) {
//    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
//    return (6.0 *mCoefficient(0,0) * normalizedTime + 2.0 * mCoefficient(1,0)) / pow(mTimeDuration, 2.0);
//}
