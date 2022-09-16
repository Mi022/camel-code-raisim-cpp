//
// Created by hwayoung on 22. 8. 29.
//

#include "IceCreamSimulation.h"
#include "IceCreamRobot.h"
#include "IceCreamTestController.h"
#include "IceCreamLQRController.h"
#include "IceCreamSharedMemory.h"

extern MainWindow *MainUI;

pthread_t thread_simulation;
pSHM sharedMemory;

std::string urdfPath = "\\home\\hwayoung\\raisimLib\\camel-code-raisim-cpp\\rsc\\camel_hard_IceCream.urdf";
std::string name = "cuteIceCream";

raisim::World world;
double simulationDuration = 4.0;
double dT = 0.005;

IceCreamSimulation sim = IceCreamSimulation(&world, dT);
IceCreamRobot robot = IceCreamRobot(&world, urdfPath, name);
//IceCreamTestController controller = IceCreamTestController(&robot, dT);
IceCreamLQRController controller = IceCreamLQRController(&robot, dT);

double oneCycleSimTime = 0;
int iteration = 0;

void realTimePlot() {
    sharedMemory->simTime = world.getWorldTime();
    sharedMemory->plotW1B = controller.torque[1];
    sharedMemory->plotW1R = 0;
    sharedMemory->plotW2B = 0;
//    sharedMemory->plotW2R = controller.torque[1];
    sharedMemory->plotW2R = 0;
    sharedMemory->plotW3B = 0;
    sharedMemory->plotW3R = 0;
}

void resetSimulationVars() {
    oneCycleSimTime = 0;
    iteration = 0;
}

void raisimSimulation() {
    realTimePlot();
    if ((MainUI->button1) && (oneCycleSimTime < simulationDuration)) {
        oneCycleSimTime = iteration * dT;
//        controller.doControl();
//        std::cout<<"tau2 : "<<controller.torque[1]<<std::endl;
//        std::cout<<"acc1 : "<<controller.measuredAcc[0]<<std::endl;
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
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) {
            std::cout << "RT Deadline Miss, Time Checker thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001
                      << " ms" << std::endl;
        }
    }
}

int main(int argc, char *argv[]) {
//    auto box = world.addBox(1, 1, 1, 10);
//    box->setPosition(3, 3, 5);
//    world.setGravity({0.0, 0.0, 0.0});
    QApplication a(argc, argv);
    MainWindow w;
    sharedMemory = (pSHM) malloc(sizeof(SHM));
    int thread_id_simulation = generate_rt_thread(thread_simulation, rt_simulation_thread, "simulation_thread", 1, 99,
                                                  NULL);
    raisim::RaisimServer server(&world);
    server.launchServer(8080);
    w.show();

    return a.exec();
}