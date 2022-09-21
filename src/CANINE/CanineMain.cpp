//
// Created by hs on 22. 8. 21.
//

#include "CanineMain.h"
#include <QApplication>

extern MainWindow *MainUI;
pthread_t thread_simulation;
pSHM smem;

std::string urdfPath = "\\home\\hs\\raisimLib\\camel-code-raisim-cpp\\rsc\\canine\\urdf\\canineV1.urdf";
std::string name = "CANINE";
raisim::World world;

double dT = 0.005;
CanineMain sim = CanineMain(&world, dT);
CanineRobot robot = CanineRobot(&world, urdfPath, name);
MPCController MPCcontroller = MPCController(&robot, dT);

double oneCycleSimTime = 0;
int iteration = 0;

void resetSimAndPlotVars() {
    iteration = 0;
    oneCycleSimTime = 0;
}

void realTimePlot(){
    smem->simTime = world.getWorldTime();
    smem->GetPosX = MPCcontroller.cmpcSolver.p[0];
    smem->DesPosX = MPCcontroller.cmpcSolver.desiredPositionX;
    smem->GetPosY = MPCcontroller.cmpcSolver.p[1];
    smem->DesPosY = MPCcontroller.cmpcSolver.desiredPositionY;
    smem->GetPosZ = MPCcontroller.cmpcSolver.p[2];
    smem->DesPosZ = MPCcontroller.cmpcSolver.desiredPositionZ;

    smem->GetRotX = MPCcontroller.cmpcSolver.q[0];
    smem->DesRotX = MPCcontroller.cmpcSolver.desiredRotationX;
    smem->GetRotY = MPCcontroller.cmpcSolver.q[1];
    smem->DesRotY = MPCcontroller.cmpcSolver.desiredRotationY;
    smem->GetRotZ = MPCcontroller.cmpcSolver.q[2];
    smem->DesRotZ = MPCcontroller.cmpcSolver.desiredRotationZ;

    smem->footPointX[0] = MPCcontroller.footPosition[0][0];
    smem->footPointX[1] = MPCcontroller.footPosition[1][0];
    smem->footPointX[2] = MPCcontroller.footPosition[2][0];
    smem->footPointX[3] = MPCcontroller.footPosition[3][0];

    smem->footPointZ[0] = MPCcontroller.footPosition[0][2];
    smem->footPointZ[1] = MPCcontroller.footPosition[1][2];
    smem->footPointZ[2] = MPCcontroller.footPosition[2][2];
    smem->footPointZ[3] = MPCcontroller.footPosition[3][2];
}

void raisimSimulation()
{
    realTimePlot();
    if (MainUI->button1)
    {
        if(MainUI->gaitChanged == 1)
        {
            std::cout << "===========Gait changed============" << std::endl;
            MPCcontroller.setGait(MainUI->gaitIdx);
            MainUI->gaitChanged = 0;
        }
        oneCycleSimTime = iteration * dT;
        MPCcontroller.doControl();
        world.integrate();
        iteration++;
    }
    else
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
//            std::cout << "RT Deadline Miss, Time Checker thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001
//                      << " ms" << std::endl;
        }

    }
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    smem = (pSHM) malloc(sizeof(SHM));

    raisim::RaisimServer server(&world);
    server.launchServer(8080);

    int thread_id_timeChecker = generate_rt_thread(thread_simulation, rt_simulation_thread, "simulation_thread", 7, 99,
                                                   NULL);

    w.show();
    return a.exec();

}
