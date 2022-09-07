//
// Created by jaehoon on 22. 5. 2.
//

#include "A1Simulation.h"
#include "mainwindow.h"
#include "include/RT/rb_utils.h"
#include "SharedMemory.h"
#include <QApplication>
#include <cmath>

extern MainWindow *MainUI;
pthread_t thread_simulation;
pSHM smem;

std::string urdfPath = "\\home\\hs\\raisimLib\\rsc\\a1\\urdf\\a1.urdf";
std::string name = "cuteA1";
raisim::World world;

double simulationDuration = 10.0;
double dT = 0.005;
A1Simulation sim = A1Simulation(&world, dT);
A1Robot robot = A1Robot(&world, urdfPath, name);
A1MPCController MPCcontroller = A1MPCController(&robot, dT);

double oneCycleSimTime = 0;
int iteration = 0;

void resetSimAndPlotVars() {
    iteration = 0;
    oneCycleSimTime = 0;
}

void realTimePlot(){
    smem->simTime = world.getWorldTime();
    smem->GetPosX = MPCcontroller.p[0];
    smem->DesPosX = MPCcontroller.desiredPositionX;
    smem->GetPosY = MPCcontroller.p[1];
    smem->DesPosY = MPCcontroller.desiredPositionY;
    smem->GetPosZ = MPCcontroller.p[2];
    smem->DesPosZ = MPCcontroller.desiredPositionZ;

    smem->GetRotX = MPCcontroller.q[0];
    smem->DesRotX = MPCcontroller.desiredRotationX;
    smem->GetRotY = MPCcontroller.q[1];
    smem->DesRotY = MPCcontroller.desiredRotationY;
    smem->GetRotZ = MPCcontroller.q[2];
    smem->DesRotZ = MPCcontroller.desiredRotationZ;

    smem->FR_hipJoint = MPCcontroller.position[7];
    smem->FL_hipJoint = MPCcontroller.position[10];
    smem->RR_hipJoint = MPCcontroller.position[13];
    smem->RL_hipJoint = MPCcontroller.position[16];

    smem->FR_thightJoint = MPCcontroller.position[8];
    smem->FL_thightJoint= MPCcontroller.position[11];
    smem->RR_thightJoint= MPCcontroller.position[14];
    smem->RL_thightJoint = MPCcontroller.position[17];

    smem->FR_calfJoint = MPCcontroller.position[9];
    smem->FL_calfJoint = MPCcontroller.position[12];
    smem->RR_calfJoint = MPCcontroller.position[15];
    smem->RL_calfJoint = MPCcontroller.position[18];
}

void raisimSimulation()
{
    realTimePlot();
    if ((MainUI->button1) && (oneCycleSimTime < simulationDuration))
    {
        oneCycleSimTime = iteration * dT;
        MPCcontroller.doControl();
        world.integrate();
        iteration++;
    }
    else if (oneCycleSimTime >= simulationDuration)
    {
        MainUI->button1 = false;
        resetSimAndPlotVars();
    }
}

void *rt_simulation_thread(void *arg)
{
    std::cout << "entered #rt_time_checker_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(dT * 1e6); // 200Hz 짜리 쓰레드

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    std::cout << "bf #while" << std::endl;
    std::cout << "control freq : " << 1 / double(PERIOD_US) * 1e6 << std::endl;

    while (true) {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW); //현재 시간 구함
        timespec_add_us(&TIME_NEXT, PERIOD_US);   //목표 시간 구함

        raisimSimulation();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL); //목표시간까지 기다림 (현재시간이 이미 오바되어 있으면 바로 넘어갈 듯)
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) {  // 현재시간이 목표시간 보다 오바되면 경고 띄우기
            std::cout << "RT Deadline Miss, Time Checker thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001
                      << " ms" << std::endl;
        }
    }
}

<<<<<<< HEAD
int main(int argc, char *argv[]) {
    std::string urdfPath = "\\home\\cha\\raisimLib\\rsc\\a1\\urdf\\a1.urdf";
    std::string name = "cuteA1";
    raisim::World world;
    double simulationDuration = 3.0;
    A1Simulation sim = A1Simulation(&world, 0.001);
    A1Robot robotA1 = A1Robot(&world, urdfPath, name);
    A1JointPDController PDcontroller = A1JointPDController(&robotA1);
=======
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    smem = (pSHM) malloc(sizeof(SHM));

    int thread_id_timeChecker = generate_rt_thread(thread_simulation, rt_simulation_thread, "simulation_thread", 0, 99,
                                                   NULL);
>>>>>>> d620b6738b25106075a1e90761133c49ca78970c

    raisim::RaisimServer server(&world);
    server.launchServer(8080);
    server.focusOn(robot.robot);

    w.show();

    return a.exec();
}


