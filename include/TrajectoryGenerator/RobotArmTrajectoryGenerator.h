//
// Created by jy on 22. 6. 1.
//

#ifndef RAISIM_ROBOTARMTRAJECTORYGENERATOR_H
#define RAISIM_ROBOTARMTRAJECTORYGENERATOR_H
#include <Eigen/Eigen>
#include <cmath>

class RobotArmTrajectoryGenerator {
public:
//    RobotArmTrajectoryGenerator()
//    {
//
//    }
    void setWaypoints(Eigen::MatrixXd);
    void caculateCoefficient();
    void updateTrajectory(double currentTime,double timeDuration);
    std::vector<double> getPositionTrajectory(double currentTime);
    double getVelocityTrajectory(double currentTime);


private:

    Eigen::MatrixXd mWaypoints;
    Eigen::MatrixXd coefficient;
    int pointNum;
    double mReferenceTime;
    double mTimeDuration;
};


#endif //RAISIM_ROBOTARMTRAJECTORYGENERATOR_H