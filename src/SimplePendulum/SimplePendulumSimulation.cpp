//
// Created by jaehoon on 22. 3. 31..
//

#include "SimplePendulumSimulation.h"
#include "include/SimulationUI/simulationMainwindow.h"
#include "include/RT/rb_utils.h"
#include <QApplication>
#include <cmath>

extern MainWindow *MainUI;
pthread_t thread_simulation;
std::string urdfPath = "\\home\\cha\\raisimLib\\camel-code-raisim-cpp\\rsc\\camel_simple_pendulum.urdf";
std::string name = "cutePendulum";
raisim::World world;

double simulationDuration = 5.0;
double dT = 0.005;
SimplePendulumSimulation sim = SimplePendulumSimulation(&world, dT);
SimplePendulumRobot robot = SimplePendulumRobot(&world, urdfPath, name);
SimplePendulumPDController controller = SimplePendulumPDController(&robot);

void raisimSimulation() {
    // TODO : Relieve CPU. Now, CPU usage is 100% !!!!!
    double dT = world.getTimeStep();
    double oneCycleSimTime = 0;
    int divider = ceil(simulationDuration / dT / 200);
    int i = 0;
    auto begin = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    while (true) {
        if ((MainUI->button1) && (oneCycleSimTime < simulationDuration)) {
            // control robot and data plot thread
            if(i == 0){begin = std::chrono::high_resolution_clock::now();}
            oneCycleSimTime = i * dT;
            controller.doControl();
            world.integrate();
            if (i % divider == 0) {
                //                std::cout<<"data_idx : "<<MainUI->data_idx<<std::endl;
                MainUI->data_x[MainUI->data_idx] = world.getWorldTime();
                MainUI->data_y1[MainUI->data_idx] = robot.getQ();
                MainUI->data_y1_desired[MainUI->data_idx] = controller.desiredPosition;
                MainUI->data_y2[MainUI->data_idx] = robot.getQD();
                MainUI->data_y2_desired[MainUI->data_idx] = controller.desiredVelocity;
//                MainUI->data_y2[MainUI->data_idx] = controller.torque[0];
                MainUI->data_idx += 1;
            }
            i++;
        } else if (oneCycleSimTime >= simulationDuration) {
            end = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
            std::cout <<" Time measured: "<< elapsed.count() * 1e-9 <<"seconds" <<std::endl;
            MainUI->button1 = false;
            i = 0;
            oneCycleSimTime = 0;
            MainUI->plotWidget1();
            MainUI->plotWidget2();
            MainUI->data_idx = 0;
        }
    }
}

void *rt_simulation_thread(void *arg) {
    std::cout << "entered #rt_time_checker_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(dT * 1e6); // 200Hz 짜리 쓰레드

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    std::cout << "bf #while" << std::endl;
    std::cout << "control freq : "<< 1/double(PERIOD_US) *1e6 << std::endl;
    while (true) {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW); //현재 시간 구함
        timespec_add_us(&TIME_NEXT, PERIOD_US);   //목표 시간 구함

        raisimSimulation();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL); //목표시간까지 기다림 (현재시간이 이미 오바되어 있으면 바로 넘어갈 듯)
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) {  // 현재시간이 목표시간 보다 오바되면 경고 띄우기
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

