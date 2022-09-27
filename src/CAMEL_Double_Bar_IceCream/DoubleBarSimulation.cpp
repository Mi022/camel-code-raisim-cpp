//
// Created by hwayoung on 22. 8. 29.
//

#include "DoubleBarSimulation.h"
#include "DoubleBarRobot.h"
#include "DoubleBarTestController.h"
#include "DoubleBarRBDLController.h"
#include "DoubleBarLQRController.hpp"
#include "DoubleBarSharedMemory.h"

extern MainWindow *MainUI;

pthread_t thread_simulation;
pSHM sharedMemory;

std::string urdfPath = "/home/hwayoung/raisimLib/camel-code-raisim-cpp/rsc/camel_double_bar_IceCream.urdf";
std::string name = "cuteIceCream";

raisim::World world;
double simulationDuration = 5.0;
double dT = 0.005;

DoubleBarSimulation sim = DoubleBarSimulation(&world, dT);
DoubleBarRobot robot = DoubleBarRobot(&world, urdfPath, name);
//DoubleBarTestController controller = DoubleBarTestController(&robot);
//DoubleBarRBDLController controller = DoubleBarRBDLController(&robot, urdfPath);
DoubleBarLQRController controller = DoubleBarLQRController(&robot, dT, urdfPath);
double oneCycleSimTime = 0;
int iteration = 0;

void realTimePlot() {
    sharedMemory->simTime = world.getWorldTime();
    sharedMemory->jointPosition = 0;
    sharedMemory->jointVelocity = 0;
    sharedMemory->jointTorque = 0;
    sharedMemory->desiredJointPosition = 0;
    sharedMemory->desiredJointVelocity = 0;
}

void resetSimulationVars() {
    oneCycleSimTime = 0;
    iteration = 0;
}

void raisimSimulation() {
    realTimePlot();
    if ((MainUI->button1) && (oneCycleSimTime < simulationDuration)) {
        oneCycleSimTime = iteration * dT;
        controller.doControl();
        world.integrate();
        iteration++;
    } else if (oneCycleSimTime >= simulationDuration) {
        MainUI->button1 = false;
        resetSimulationVars();
    }
}

void *rt_simulation_thread(void *arg) {
    std::cout << "entered #rt_simulation_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(dT * 1e6);

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    std::cout << "bf #while" << std::endl;
    std::cout << "control freq : " << 1 / double(PERIOD_US) * 1e6 << std::endl;
    while (true) {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        raisimSimulation();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL); //목표시간까지 기다림 (현재시간이 이미 오바되어 있으면 바로 넘어갈 듯)
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) {
            std::cout << "RT Deadline Miss, Time Checker thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001
                      << " ms" << std::endl;
        }
    }
}

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    MainWindow w;
    sharedMemory = (pSHM) malloc(sizeof(SHM));
//    sim = DoubleBarSimulation(&world, dT);
//    robot = DoubleBarRobot(&world, urdfPath, name);
//    controller = DoubleBarLQRController(&robot, dT, urdfPath);
    int thread_id_simulation = generate_rt_thread(thread_simulation, rt_simulation_thread, "simulation_thread", 1, 99,
                                                  NULL);
    raisim::RaisimServer server(&world);
//    world.setGravity({0.0, 0.0, 0.0});
    server.launchServer(8080);
    w.show();

    return a.exec();
}