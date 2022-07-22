//
// Created by hs on 22. 7. 14.
//

#ifndef RAISIM_LEGTRAJECTORY_H
#define RAISIM_LEGTRAJECTORY_H

#include <Eigen/Eigen>
#include <cmath>

class legtrajectory {
public:
    void updateTrajectory(double currentPosition, double currentTime,double timeDuration);
    double getPositionTrajectory(double currentTime);

private:
    double mReferencePose;
    double mReferenceTime;
    double mTimeDuration;

};


#endif //RAISIM_LEGTRAJECTORY_H
