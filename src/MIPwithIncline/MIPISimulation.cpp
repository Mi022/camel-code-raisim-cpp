//
// Created by user on 22. 6. 18.
//

#include "MIPISimulation.h"
#include "include/SimulationUI/simulationMainwindow.h"
#include "include/RT/rb_utils.h"
#include "MIPILQRController.h"
#include <QApplication>
#include <cmath>

#include <fstream>

extern MainWindow *MainUI;
pthread_t thread_simulation;

std::string urdfPath = "\\home\\user\\raisimLib\\camel-code-raisim-cpp\\rsc\\camel_momentum_inverted_pendulum_with_incline.urdf";
std::string name = "MIPI";
raisim::World world;

std::ofstream myfile1;
std::ofstream myfile2;
std::ofstream myfile3;

double simulationDuration = 20.0;
double dT = 0.005;
MIPISimulation sim = MIPISimulation(&world, dT);
MIPIRobot robot = MIPIRobot(&world, urdfPath, name);
MIPILQRController controller = MIPILQRController(&robot, &world);


double oneCycleSimTime = 0;
int divider = ceil(simulationDuration / dT / 200);
int iteration = 0;

void raisimSimulation() {
    if ((MainUI->button1) && (oneCycleSimTime < simulationDuration)) {
        oneCycleSimTime = iteration * dT;
        controller.doControl();
        std::cout<<"test"<<std::endl;
        world.integrate();
        if (iteration % divider == 0) {
            MainUI->data_x[MainUI->data_idx] = world.getWorldTime();
            MainUI->data_y1[MainUI->data_idx] = robot.getQ()[0];
            MainUI->data_y1_desired[MainUI->data_idx] = 0;
            MainUI->data_y2[MainUI->data_idx] = robot.getQD()[0];
            MainUI->data_y2_desired[MainUI->data_idx] = 0;
            MainUI->data_y3_blue[MainUI->data_idx] = controller.torque[2];

//            myfile1 << robot.getQ()[0] <<"," ;
//            myfile2 << robot.getQD()[0] <<"," ;
//            myfile3 << robot.getQD()[1] <<"," ;
            MainUI->data_y3_red[MainUI->data_idx] = robot.getQD()[2];
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
        MainUI->data_idx = 0;
//        myfile1.close();
//        myfile2.close();
//        myfile3.close();
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
//    myfile1.open ("rodAngleLQR.csv");
//    myfile2.open ("rodAngularVelocityLQR.csv");
//    myfile3.open ("motorAngularVelocityLQR.csv");
    QApplication a(argc, argv);
    MainWindow w;
    raisim::RaisimServer server(&world);
    server.launchServer(8080);
    int thread_id_timeChecker = generate_rt_thread(thread_simulation, rt_simulation_thread, "simulation_thread", 0, 99,
                                                   NULL);
    w.show();

    return a.exec();
}

