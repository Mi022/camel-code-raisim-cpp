//
// Created by jaehoon on 22. 3. 31.. dkswogns46@gmail.com
//

#include "SingleLeggedSimulation.h"
#include "SingleLeggedSharedMemory.h"
#include "UI/simulationMainwindow.h"
#include "include/RT/rb_utils.h"
#include <QApplication>
#include <cmath>
#include "A1CollisionDetecter.h"

extern MainWindow *MainUI;
pthread_t thread_simulation;
pSHM sharedMemory;

std::string urdfPath = "\\home\\cha\\git\\repository-group\\raisimLib\\camel-code-raisim-cpp\\rsc\\test_a1_single_leg_right\\camel_single_leg.urdf";

std::string name = "single_leg";
raisim::World world;

double simulationDuration = 10.0;
double dT = 0.005;
SingleLeggedSimulation sim = SingleLeggedSimulation(&world, dT);
SingleLeggedRobot robot = SingleLeggedRobot(&world, urdfPath, name);

//SingleLeggedPDController controller = SingleLeggedPDController(&robot);
//SingleLeggedIDController controller = SingleLeggedIDController(&robot, dT);
SingleLeggedMPCController controller = SingleLeggedMPCController(&robot, dT);
//SingleLeggedMPCqpoases controller = SingleLeggedMPCqpoases(&robot, dT);

/// for momentum observer
A1CollisionDetecter FrontRightExteranlTorqueObserver;
Eigen::VectorXd momentum = Eigen::VectorXd(2);
Eigen::VectorXd momentumPrev = Eigen::VectorXd(2);
Eigen::VectorXd residual = Eigen::VectorXd(2);
Eigen::VectorXd beta = Eigen::VectorXd(2);
Eigen::VectorXd dqMat = Eigen::VectorXd(2);
Eigen::Matrix2d gainK;
Eigen::Vector2d tempTorque;
bool firstRun = true;

double oneCycleSimTime = 0;
int divider = ceil(simulationDuration / dT / 200);
int iteration = 0;
struct timespec TIME_TIC;
struct timespec TIME_TOC;

void updateSHM() {
    sharedMemory->time = world.getWorldTime();
    sharedMemory->position_z = controller.position[0];
    sharedMemory->desiredPosition_z = controller.desiredPosition;
    sharedMemory->velocity_z = controller.velocity[0];
    sharedMemory->desiredVelocity_z = controller.desiredVelocity;
    sharedMemory->jointPosition[0] = controller.position[1];
    sharedMemory->jointPosition[1] = controller.position[2];
    sharedMemory->jointVelocity[0] = controller.velocity[1];
    sharedMemory->jointVelocity[1] = controller.velocity[2];
    sharedMemory->jointTorque[0] = controller.torque[0];
    sharedMemory->jointTorque[1] = controller.torque[1];

}

void resetSimVarialbes() {
    iteration = 0;
    oneCycleSimTime = 0;
}

void raisimSimulation() {
    if ((MainUI->button1) && (oneCycleSimTime < simulationDuration)) {
        oneCycleSimTime = iteration * dT;

        clock_gettime(CLOCK_REALTIME, &TIME_TIC);
        controller.doControl();
        //sharedMemory->beta = FrontRightExteranlTorqueObserver.Beta(robot.getQ()[1], robot.getQD()[1], robot.getQ()[2], robot.getQD()[2]);
        beta = FrontRightExteranlTorqueObserver.Beta(robot.getQ()[1], robot.getQD()[1], robot.getQ()[2], robot.getQD()[2]);
        clock_gettime(CLOCK_REALTIME, &TIME_TOC);
        //std::cout << "MPC calculation time : " << timediff_us(&TIME_TIC, &TIME_TOC) * 0.001 << " ms" << std::endl;
//        std::cout << robot.getQ()[0]<<", "<< robot.getQ()[1]*180/3.141592<<", "<< robot.getQ()[2]*180/3.141592<< std::endl;
        robot.robot->setExternalForce(robot.robot->getBodyIdx("lower_leg"),{0,0,-0.1},{-20,0,-20});
        world.integrate();
        updateSHM();
        tempTorque[0] = controller.torque[1];
        tempTorque[1] = controller.torque[2];
        iteration++;

        //std::cout << sharedMemory->firstRun << std::endl;

        if (firstRun == true) {
            momentumPrev[0] = 0;
            momentumPrev[1] = 0;
            residual[0] = 0;
            residual[1] = 0;
            firstRun = false;
        }

        dqMat[0] = robot.getQD()[1];
        dqMat[1] = robot.getQD()[2];
        momentum = momentumPrev + tempTorque*dT - beta*dT + residual*dT;
        residual = gainK*(-momentum + FrontRightExteranlTorqueObserver.MassMat(sharedMemory->jointPosition[0],sharedMemory->jointPosition[1])*dqMat);
        momentumPrev = momentum;
//        FrontRightExteranlTorqueObserver.MassMat(robot.getQ()[1],robot.getQ()[2]);
        std::cout << "joint 1 : "<<residual[0] << " joint 2 : " << residual[1] <<" torque1 :" <<controller.torque[1] <<" torque2 :" <<controller.torque[2] <<" beta 1 : "<<beta[0]<<" beta 2 : "<<beta[1] <<std::endl;


    } else if (oneCycleSimTime >= simulationDuration) {
        MainUI->button1 = false;
        MainUI->isSimulationEnd = true;
        resetSimVarialbes();
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
    sharedMemory = (pSHM) malloc(sizeof(SHM));
    sim.setGroundProperty("wheat");
    raisim::RaisimServer server(&world);
    server.launchServer(8080);
    sharedMemory->custom = 0.0;

    gainK(0,0) = 100;
    gainK(0,1) = 0;
    gainK(1,0) = 0;
    gainK(1,1) = 100;

    controller.setFrontLegTorqueObserver(&FrontRightExteranlTorqueObserver);

    int thread_id_timeChecker = generate_rt_thread(thread_simulation, rt_simulation_thread, "simulation_thread", 0, 99,
                                                   NULL);
    w.show();

    return a.exec();
}
