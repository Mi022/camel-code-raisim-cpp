//
// Created by hs on 22. 7. 14.
//

#ifndef RAISIM_LEGTRAJECTORY_H
#define RAISIM_LEGTRAJECTORY_H

#define PNUM 12

#include <Eigen/Eigen>
#include <cmath>

class legtrajectory {
public:
    void updateTrajectory(double currentTime,double timeDuration);
    void getPositionTrajectory(double currentTime);
    double factorial(double value);

    double sumX = 0.0;
    double sumZ = 0.0;

private:
    double mReferenceTime;
    double mTimeDuration;

    double pz[PNUM] = {-0.37, -0.37,
                       -0.29, -0.29, -0.29, -0.29, -0.29,
                       -0.27, -0.27, -0.27,
                       -0.37, -0.37};
    double px[PNUM] = {0.0, -0.08,
                       -0.1, -0.1, -0.1, 0.0, 0.0,
                       0.0, 0.1, 0.1,
                       0.08, 0.0};
};


#endif //RAISIM_LEGTRAJECTORY_H
