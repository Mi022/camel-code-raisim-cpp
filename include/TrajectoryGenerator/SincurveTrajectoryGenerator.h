//
// Created by hs on 22. 6. 24.
//

#ifndef RAISIM_SINCURVETRAJECTORYGENERATOR_H
#define RAISIM_SINCURVETRAJECTORYGENERATOR_H

class SincurveTrajectoryGenerator{
public:
    SincurveTrajectoryGenerator()
    {

    }
    void updateTrajectory(double currentPosition, double currentTime,double timeDuration);
    double getPositionTrajectory(double currentTime);
    double getVelocityTrajectory(double currentTime);

private:
    double mReferencePose;
    double mReferenceTime;
    double mTimeDuration;
};

#endif //RAISIM_SINCURVETRAJECTORYGENERATOR_H
