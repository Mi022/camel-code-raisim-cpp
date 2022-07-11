//
// Created by jy on 22. 7. 5.
//

#ifndef RAISIM_ROBOTARMWAYPOINTS_H
#define RAISIM_ROBOTARMWAYPOINTS_H


class RobotArmWayPoints {

public:
    double setObstalce(double *, double (*obstacle_center)[3]);
private:
    double mObstacleRadius[2];
    double mObstacleCenter[][3];
};


#endif //RAISIM_ROBOTARMWAYPOINTS_H
