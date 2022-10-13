//
// Created by hs on 22. 8. 21.
//

#include "CanineMain.h"
#include <QApplication>

#include "collisionDetector/CanineExternalTorqueObserver.hpp"

extern MainWindow *MainUI;
pthread_t thread_simulation;
pSHM smem;

std::string urdfPath = "\\home\\cha\\git\\repository-group\\raisimLib\\camel-code-raisim-cpp\\rsc\\canine\\urdf\\canineV1.urdf";
std::string name = "CANINE";
raisim::World world;

double dT = 0.005;
CanineMain sim = CanineMain(&world, dT);
CanineRobot robot = CanineRobot(&world, urdfPath, name);
MPCController MPCcontroller = MPCController(&robot, dT);

///for estimating velocity
double velX;
double prevPosX = 0;
Eigen::Matrix4d InvTkf;
Eigen::Matrix4d InvTtk;
Eigen::Matrix4d InvTht;
Eigen::Matrix4d InvTbh;
Eigen::Matrix4d Tkf;
Eigen::Matrix4d Ttk;
Eigen::Matrix4d Tht;
Eigen::Matrix4d Tbh;
Eigen::Matrix4d RightFrontFoot2Base;
Eigen::Matrix4d TEST;



///external torque observer
CanineExternalTorqueObserver RearRightExternalTorqueObserver;

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
        RearRightExternalTorqueObserver.UpdateBeta(robot.getQ()[14],robot.getQD()[13],robot.getQ()[15],robot.getQD()[14]);
        world.integrate();
        iteration++;
        if(*(MPCcontroller.GetMpcTable()+1)==0) //
        {
            RearRightExternalTorqueObserver.UpdateState(robot.getQ(),robot.getQD(),MPCcontroller.GetTorque()[13],MPCcontroller.GetTorque()[14]);
//            std::cout << "joint 1 : " << RearRightExternalTorqueObserver.GetResidualVector()[0] << " joint 2 : " << RearRightExternalTorqueObserver.GetResidualVector()[1] << " torque1 :" << MPCcontroller.GetTorque()[13] << " torque2 :" << MPCcontroller.GetTorque()[14] <<" beta1 : "<<RearRightExternalTorqueObserver.GetBeta()[0] <<std::endl;
            if(RearRightExternalTorqueObserver.GetResidualVector()[1] > 5)
            {
                MPCcontroller.setGait(0);
            }

        }


//        std::cout <<"pos.x : " << robot.robot->getGeneralizedCoordinate()[0] << " pos.y : " << robot.robot->getGeneralizedCoordinate()[1] << " pos.z : " << robot.robot->getGeneralizedCoordinate()[2] << std::endl;
//        velX = (robot.robot->getGeneralizedCoordinate()[0] - prevPosX)/dT;
//        prevPosX = robot.robot->getGeneralizedCoordinate()[0];
//        std::cout << velX <<" " <<robot.robot->getGeneralizedVelocity()[0]<<std::endl;



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
            //std::cout << "RT Deadline Miss, Time Checker thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001
            //          << " ms" << std::endl;
        }
    }
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    smem = (pSHM) malloc(sizeof(SHM));

    world.addArticulatedSystem("\\home\\cha\\git\\repository-group\\raisimLib\\camel-code-raisim-cpp\\rsc\\obstacle.urdf");
    raisim::RaisimServer server(&world);
    server.launchServer(8080);

    int thread_id_timeChecker = generate_rt_thread(thread_simulation, rt_simulation_thread, "simulation_thread", 7, 99,
                                                   NULL);

    w.show();
    return a.exec();

}
