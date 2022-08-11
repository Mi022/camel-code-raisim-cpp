#include "SimplePendulumSimulation.h"     //include Simulation, Robot, Controller
#include "SimplePendulumRobot.h"
#include "SimplePendulumPDController.h"
#include "SimplePendulumSharedMemory.h"

// for UI
extern MainWindow *MainUI;
// for RT thread
pthread_t thread_simulation;
pSHM sharedMemory;

// robot's urdfPath and name. IT SHOULD BE CHANGED TO YOUR PATH.
std::string urdfPath = "\\home\\jaehyeong\\raisimLib\\camel-code-raisim-cpp\\rsc\\camel_simple_pendulum.urdf";
std::string name = "cutePendulum";

// raisim world
raisim::World world;

// Simulation running time when RUN button is pressed once.
double simulationDuration = 5.0;

// discrete time of simulation
double dT = 0.005;

// declare simulation, robot and controller
SimplePendulumSimulation sim = SimplePendulumSimulation(&world, dT);
SimplePendulumRobot robot = SimplePendulumRobot(&world, urdfPath, name);
SimplePendulumPDController controller = SimplePendulumPDController(&robot);

// other variables
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
    server.launchServer(8080);
    w.show();

    return a.exec();
}