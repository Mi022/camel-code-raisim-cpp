#include "include/CAMEL/Simulation.h"   //include basic sources
#include "UI/simulationMainwindow.h"
#include "include/RT/rb_utils.h"
#include <QApplication>

// extend Simulation class (parent) to SimplePendulumSimulation class (child)
class SimplePendulumSimulation : public Simulation {

public:
    //initialize
    SimplePendulumSimulation(raisim::World *world, double dT) : Simulation(world, dT) { ; }

private:

};