//
// Created by jaehoon on 22. 5. 2.
//

#include "A1Simulation.h"
#include "mainwindow.h"
#include "include/RT/rb_utils.h"
#include <QApplication>
#include <cmath>

extern MainWindow *MainUI;
pthread_t thread_simulation;

//std::string urdfPath = "\\home\\hs\\raisimLib\\camel-code-raisim-cpp\\rsc\\camel_SRB_quad.urdf";
std::string urdfPath = "\\home\\hs\\raisimLib\\rsc\\a1\\urdf\\a1.urdf";
std::string name = "cuteA1";
raisim::World world;

double simulationDuration = 5.0;
double dT = 0.005;
A1Simulation sim = A1Simulation(&world, dT);
A1Robot robot = A1Robot(&world, urdfPath, name);

A1MPCController MPCcontroller = A1MPCController(&robot, dT);
A1JointPDController PDcontroller = A1JointPDController(&robot);

double oneCycleSimTime = 0;
int divider = ceil(simulationDuration / dT / 200);
int iteration = 0;
void raisimSimulation() {
    if ((MainUI->button1) && (oneCycleSimTime < simulationDuration)) {
        oneCycleSimTime = iteration * dT;
        MPCcontroller.doControl();
        world.integrate();
        if (iteration % divider == 0) {
            MainUI->data_x[MainUI->data_idx] = world.getWorldTime();
            MainUI->data_y1[MainUI->data_idx] = MPCcontroller.p[0];
            MainUI->data_y1_desired[MainUI->data_idx] = MPCcontroller.desiredPositionX;
            MainUI->data_y2[MainUI->data_idx] = MPCcontroller.p[1];
            MainUI->data_y2_desired[MainUI->data_idx] = MPCcontroller.desiredPositionY;
            MainUI->data_y3[MainUI->data_idx] = MPCcontroller.p[2];
            MainUI->data_y3_desired[MainUI->data_idx] = MPCcontroller.desiredPositionZ;

            MainUI->data_y4[MainUI->data_idx] = MPCcontroller.q[0];
            MainUI->data_y4_desired[MainUI->data_idx] = MPCcontroller.desiredRotationX;
            MainUI->data_y5[MainUI->data_idx] = MPCcontroller.q[1];
            MainUI->data_y5_desired[MainUI->data_idx] = MPCcontroller.desiredRotationY;
            MainUI->data_y6[MainUI->data_idx] = MPCcontroller.q[2];
            MainUI->data_y6_desired[MainUI->data_idx] = MPCcontroller.desiredRotationZ;

            MainUI->data_hip_fr[MainUI->data_idx] = MPCcontroller.position[7];
            MainUI->data_hip_fl[MainUI->data_idx] = MPCcontroller.position[10];
            MainUI->data_hip_rr[MainUI->data_idx] = MPCcontroller.position[13];
            MainUI->data_hip_rl[MainUI->data_idx] = MPCcontroller.position[16];

            MainUI->data_thigh_fr[MainUI->data_idx] = MPCcontroller.position[8];
            MainUI->data_thigh_fl[MainUI->data_idx] = MPCcontroller.position[11];
            MainUI->data_thigh_rr[MainUI->data_idx] = MPCcontroller.position[14];
            MainUI->data_thigh_rl[MainUI->data_idx] = MPCcontroller.position[17];

            MainUI->data_calf_fr[MainUI->data_idx] = MPCcontroller.position[9];
            MainUI->data_calf_fl[MainUI->data_idx] = MPCcontroller.position[12];
            MainUI->data_calf_rr[MainUI->data_idx] = MPCcontroller.position[15];
            MainUI->data_calf_rl[MainUI->data_idx] = MPCcontroller.position[18];

            MainUI->data_idx += 1;
        }
        iteration++;
    } else if (oneCycleSimTime >= simulationDuration) {
        MainUI->button1 = false;
        iteration = 0;
        oneCycleSimTime = 0;
        MainUI->plotWidget1();
        MainUI->plotWidget2();
        MainUI->plotWidget3();

        MainUI->plotWidget4();
        MainUI->plotWidget5();
        MainUI->plotWidget6();

        MainUI->plotWidget7();
        MainUI->plotWidget8();
        MainUI->plotWidget9();
        MainUI->data_idx = 0;
    }
}

void *rt_simulation_thread(void *arg) {
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

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    MainWindow w;
    raisim::RaisimServer server(&world);
    server.launchServer(8080);
    server.focusOn(robot.robot);


    int thread_id_timeChecker = generate_rt_thread(thread_simulation, rt_simulation_thread, "simulation_thread", 0, 99,
                                                   NULL);
    w.show();

    return a.exec();
}


