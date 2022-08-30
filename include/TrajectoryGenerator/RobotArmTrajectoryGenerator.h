//
// Created by jy on 22. 6. 1.
//

#ifndef RAISIM_ROBOTARMTRAJECTORYGENERATOR_H
#define RAISIM_ROBOTARMTRAJECTORYGENERATOR_H
#include <Eigen/Eigen>
#include <cmath>

class RobotArmTrajectoryGenerator {
public:

    RobotArmTrajectoryGenerator()
    {
        mMatrixA << 2.0, -2.0, 1.0, 1.0, -3.0, 3.0, -2.0, -1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    }
    void setWaypoints(Eigen::MatrixXd);
    void calculateCoefficient();
    void updateTrajectory(double currentTime,double timeDuration);
    std::vector<double> getPositionTrajectory(double currentTime);
    std::vector<double> getVelocityTrajectory(double currentTime);


private:
    Eigen::MatrixXd mMatrixA = Eigen::MatrixXd(4,4);
    Eigen::MatrixXd mWaypoints;
    Eigen::MatrixXd mCoefficient = Eigen::MatrixXd(4, 6);
    Eigen::MatrixXd mFunctionValue = Eigen::MatrixXd(4, 6);
    int pointNum;
    double mReferenceTime;
    double mTimeDuration;
};


#endif //RAISIM_ROBOTARMTRAJECTORYGENERATOR_H