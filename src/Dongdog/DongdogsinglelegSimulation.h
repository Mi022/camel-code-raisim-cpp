//
// Created by jaehoon on 22. 3. 31..
//

#ifndef RAISIM_DONGDOGSINGLELEGSIMULATION_H
#define RAISIM_DONGDOGSINGLELEGSIMULATION_H

#include "include/CAMEL/Simulation.h"
#include "DongdogsinglelegRobot.h"
#include "DongdogsinglelegPDController.h"


//


class DongdogsinglelegSimulation : public Simulation {

public:

    DongdogsinglelegSimulation(raisim::World *world, double dT) : Simulation(world, dT) { ; }

private:

};


#endif //RAISIM_SIMPLEPENDULUMSIMULATION_H
