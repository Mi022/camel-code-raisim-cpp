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
        mMatrixA << 2.0, -2.0, 1.0, 1.0,
                    -3.0, 3.0, -2.0, -1.0,
                    0.0, 0.0, 1.0, 0.0,
                    1.0, 0.0, 0.0, 0.0;
    }
    void setWaypoints(Eigen::MatrixXd);
    void updateTrajectory(double currentPosition,double goalPosition,double currentTime,double timeDuration);
    void calculateCoefficient();
    std::vector<double> getPositionTrajectory(double currentTime);
    double getVelocityTrajectory(double currentTime);
    double getAccelerationTrajectory(double currentTime);

private:
    Eigen::MatrixXd mMatrixA = Eigen::MatrixXd(4, 4);
    Eigen::MatrixXd mCoefficient = Eigen::MatrixXd(4, 1);
    Eigen::MatrixXd mFunctionValue = Eigen::MatrixXd(4, 1);
    Eigen::MatrixXd mWaypoints= Eigen::MatrixXd(3, 6);;
    double mReferenceTime;
    double mTimeDuration;
};


#endif //RAISIM_ROBOTARMTRAJECTORYGENERATOR_H