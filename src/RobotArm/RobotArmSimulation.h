//
// Created by jy on 22. 7. 4.
//

#ifndef RAISIM_ROBOTARMSIMULATION_H
#define RAISIM_ROBOTARMSIMULATION_H
#include "include/CAMEL/Simulation.h"
#include "RobotArmRobot.h"
#include "RobotArmPDController.h"
#include "Eigen/Eigen"
using namespace Eigen;

class RobotArmSimulation : public Simulation {

public:

    RobotArmSimulation(raisim::World *world, double dT) : Simulation(world, dT) {
        obstacleRadius << 0.5, 0.2;
        obstacleCenter <<  -1,  0,  0.5, 0,  1,  0.2; }


private:

    VectorXd obstacleRadius = VectorXd(2);
    MatrixXd obstacleCenter = MatrixXd(2,3);

};


#endif //RAISIM_ROBOTARMSIMULATION_H
