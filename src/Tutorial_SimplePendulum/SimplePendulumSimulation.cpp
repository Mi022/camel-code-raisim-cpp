#include "SimplePendulumSimulation.h"     //include Simulation, Robot, Controller
#include "SimplePendulumRobot.h"
#include "SimplePendulumPDController.h"

// for UI
extern MainWindow *MainUI;
// for RT thread
pthread_t thread_simulation;

// robot's urdfPath and name. IT SHOULD BE CHANGED TO YOUR PATH.
std::string urdfPath = "\\home\\camel\\raisimLib\\camel-code-raisim-cpp\\rsc\\camel_simple_pendulum.urdf";
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
int divider = ceil(simulationDuration / dT / 200);
int iteration = 0;

void plot() {
    MainUI->plotWidget1();
    MainUI->plotWidget2();
    MainUI->plotWidget3();
}

// You can change the plotted data in UI
void updatePlotData() {
    // x-axis of graph
    MainUI->data_x[MainUI->data_idx] = world.getWorldTime();
    // first graph
    MainUI->data_y1[MainUI->data_idx] = robot.getQ()[0];
    MainUI->data_y1_desired[MainUI->data_idx] = controller.desiredPosition;
    // second graph
    MainUI->data_y2[MainUI->data_idx] = robot.getQD()[0];
    MainUI->data_y2_desired[MainUI->data_idx] = controller.desiredVelocity;
    // third graph
    MainUI->data_y3_blue[MainUI->data_idx] = controller.torque[0];
    MainUI->data_idx += 1;
}

void resetSimAndPlotVars() {
    MainUI->data_idx = 0;
    iteration = 0;
    oneCycleSimTime = 0;
}

void raisimSimulation() {
    if ((MainUI->button1) && (oneCycleSimTime < simulationDuration)) {
        oneCycleSimTime = iteration * dT;
        controller.doControl();
        world.integrate();
        if (iteration % divider == 0) {
            updatePlotData();
        }
        iteration++;
    } else if (oneCycleSimTime >= simulationDuration) {
        MainUI->button1 = false;
        plot();
        resetSimAndPlotVars();
    }
}

void *rt_simulation_thread(void *arg) {
    std::cout << "entered #rt_time_checker_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(dT * 1e6);

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    std::cout << "bf #while" << std::endl;
    std::cout << "control freq : "<< 1/double(PERIOD_US) *1e6 << std::endl;
    while (true) {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);

        raisimSimulation();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL); //목표시간까지 기다림 (현재시간이 이미 오바되어 있으면 바로 넘어갈 듯)
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) {
            std::cout << "RT Deadline Miss, Time Checker thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
        }
    }
}

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    MainWindow w;
    int thread_id_timeChecker = generate_rt_thread(thread_simulation, rt_simulation_thread, "simulation_thread", 0, 99, NULL);
    raisim::RaisimServer server(&world);
    server.launchServer(8080);
    w.show();

    return a.exec();
}