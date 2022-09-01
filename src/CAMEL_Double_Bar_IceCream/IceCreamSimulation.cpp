//
// Created by hwayoung on 22. 8. 29.
//

#include "IceCreamSimulation.h"
#include "IceCreamRobot.h"
#include "IceCreamTestController.h"
#include "IceCreamSharedMemory.h"

extern MainWindow *MainUI;

pthread_t thread_simulation;
pSHM sharedMemory;

std::string urdfPath = "\\home\\hwayoung\\raisimLib\\camel-code-raisim-cpp\\rsc\\camel_double_bar_IceCream.urdf";
std::string name = "cuteIceCream";

raisim::World world;
double simulationDuration = 0.005;
double dT = 0.005;

IceCreamSimulation sim = IceCreamSimulation(&world, dT);
IceCreamRobot robot = IceCreamRobot(&world, urdfPath, name);
IceCreamTestController controller = IceCreamTestController(&robot);

double oneCycleSimTime = 0;
int iteration = 0;

void realTimePlot() {
    sharedMemory->simTime = world.getWorldTime();
    sharedMemory->jointPosition = controller.position[1];
    sharedMemory->jointVelocity = controller.velocity[1];
    sharedMemory->jointTorque = controller.torque[1];
    sharedMemory->desiredJointPosition = controller.desiredPosition[1];
    sharedMemory->desiredJointVelocity = controller.desiredVelocity[1];
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
    int thread_id_simulation = generate_rt_thread(thread_simulation, rt_simulation_thread, "simulation_thread", 1, 99,
                                                  NULL);
    raisim::RaisimServer server(&world);
//    world.setGravity({0.0, 0.0, 0.0});
    server.launchServer(8080);
    w.show();

    return a.exec();
}