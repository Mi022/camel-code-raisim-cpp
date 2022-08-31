//
// Created by jy on 22. 7. 4.
//

#include "RobotArmSimulation.h"
#include "include/SimulationUI/simulationMainwindow.h"
#include "include/RT/rb_utils.h"
#include <QApplication>
#include <cmath>
#include "src/RobotArm/RobotArmMotionPlanning.h"
#include "src/RobotArm/RobotArmCollisionChecker.h"
#include "Eigen/Eigen"

extern MainWindow *MainUI;
pthread_t thread_simulation;
std::string urdfPath = "\\home\\jy\\raisimLib\\camel-code-raisim-cpp\\rsc\\robotArm\\urdf\\kinova.urdf";
std::string urdfPath_1 = "\\home\\jy\\raisimLib\\camel-code-raisim-cpp\\rsc\\robotArm\\urdf\\kinova_1.urdf";
std::string name = "robotArm";
raisim::World world;


RobotArmCollisionChecker robotArmCollisionChecker;
Eigen::VectorXd obstacleRadius = Eigen::VectorXd(3);
Eigen::MatrixXd obstacleCenter = Eigen::MatrixXd(3,3);

double simulationDuration = 5.0;
double dT = 0.005;
double d2r = 3.141592/180;
RobotArmSimulation sim = RobotArmSimulation(&world, dT);
RobotArmRobot robot = RobotArmRobot(&world, urdfPath, name);
RobotArmRobot robot_1 = RobotArmRobot(&world, urdfPath_1, "robotArmEnd");
RobotArmPDController controller = RobotArmPDController(&robot);
RobotArmMotionPlanning motionPlanner(&robotArmCollisionChecker,controller.getTrajectoryGenerator());

double oneCycleSimTime = 0;
int divider = ceil(simulationDuration / dT / 200);
int iteration = 0;

void raisimSimulation() {

    // TODO : Relieve CPU. Now, CPU usage is 100% !!!!!
    if ((MainUI->button1) && (oneCycleSimTime < simulationDuration)) {
        // control robot and data plot thread
        oneCycleSimTime = iteration * dT;

        controller.doControl();
        world.integrate();
        if (iteration % divider == 0) {

            MainUI->data_x[MainUI->data_idx] = world.getWorldTime();
            MainUI->data_y1[MainUI->data_idx] = robot.getQ()[3];
            MainUI->data_y1_desired[MainUI->data_idx] = controller.desiredPosition[3];
            MainUI->data_y2[MainUI->data_idx] = robot.getQD()[0];
            MainUI->data_y2_desired[MainUI->data_idx] = controller.desiredVelocity[0];
//                MainUI->data_y2[MainUI->data_idx] = controller.torque[0];
            MainUI->data_idx += 1;
        }
        iteration++;
    } else if (oneCycleSimTime >= simulationDuration) {
        MainUI->button1 = false;
        iteration = 0;
        oneCycleSimTime = 0;
        MainUI->plotWidget1();
        MainUI->plotWidget2();
        MainUI->data_idx = 0;
    }
}

void *rt_simulation_thread(void *arg) {
    std::cout << "entered #rt_time_checker_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(dT * 1e6); // 200Hz 짜리 쓰레드

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
//    std::cout << "bf #while" << std::endl;
    std::cout << "control freq : "<< 1/double(PERIOD_US) *1e6 << std::endl;
    obstacleRadius << 0.2, 0.15 , 0.15;
    obstacleCenter <<  -0.5,  0.0,  0.2,
                        0.0,  0.5,  0.15,
                        -0.1,  0.02,  0.7;
    robotArmCollisionChecker.setObstacle(obstacleRadius, obstacleCenter);

    auto obstacle1 = world.addSphere(obstacleRadius(0), 0.0);
    auto obstacle2 = world.addSphere(obstacleRadius(1), 0.0);
    auto obstacle3 = world.addSphere(obstacleRadius(2), 0.0);
    obstacle1->setPosition(obstacleCenter(0,0),obstacleCenter(0,1),obstacleCenter(0,2));
    obstacle2->setPosition(obstacleCenter(1,0),obstacleCenter(1,1),obstacleCenter(1,2));
    obstacle3->setPosition(obstacleCenter(2,0),obstacleCenter(2,1),obstacleCenter(2,2));

    motionPlanner.makeTree();
    motionPlanner.dijkstra();

//    std::cout << d2r * motionPlanner.wayPoints << std::endl;


    while (true) {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW); //현재 시간 구함
        timespec_add_us(&TIME_NEXT, PERIOD_US);   //목표 시간 구함

        raisimSimulation();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL); //목표시간까지 기다림 (현재시간이 이미 오바되어 있으면 바로 넘어갈 듯)
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) {  // 현재시간이 목표시간 보다 오바되면 경고 띄우기
//            std::cout << "RT Deadline Miss, Time Checker thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001 << " ms" << std::endl;
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