//
// Created by jaehyeong on 22. 8. 11.
//
#include "JHdogSimulation.h"
#include "JHdog.h"
#include "JHdogSharedMemory.h"

/**************TO DO **************
 * 1. add controller
 * 2. urdf collision body
 * 3. be used to UI
 * 4. synchronize RMDx6
 * 5. MCTS
 *
 */

//for UI
extern MainWindow *MainUI;
// for RT thread
pthread_t thread_simulation;
pSHM sharedMemory;

std::string urdfPath = "\\home\\jaehyeong\\raisimLib\\camel-code-raisim-cpp\\rsc\\camel_JHdog_single_leg\\JHdog.urdf";
std::string name = "JHdog";

raisim::World world;

// Simulation running thime when RUN button is pressed once.
double simulationDuration = 5.0;

// discrete time of simulation
double dT = 0.005;

//declare simulation, robot and controller
JHdogSimulation sim = JHdogSimulation(&world, dT);
JHdog robot = JHdog(&world, urdfPath, name);
// controller should be added

//other variables
double oneCycleSimTime = 0;
int iteration = 0;

// You can change the plotted data in UI
void realTimePlot() {
    sharedMemory->simTime = world.getWorldTime();
    sharedMemory->jointPosition = controller.position[0];
    sharedMemory->jointVelocity = controller.velocity[0];
    sharedMemory->jointTorque = controller.torque[0];
    sharedMemory->desiredJointPosition = controller.desiredPosition;
    sharedMemory->desiredJointVelocity = controller.desiredVelocity;
}
