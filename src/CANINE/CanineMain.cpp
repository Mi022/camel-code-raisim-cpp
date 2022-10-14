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
double alpha = 0;
double beta = 0;
Eigen::Matrix4d InvTkf;
Eigen::Matrix4d InvTtk;
Eigen::Matrix4d InvTht;
Eigen::Matrix4d InvTbh;
Eigen::Matrix4d Tkf;
Eigen::Matrix4d Ttk;
Eigen::Matrix4d Tht;
Eigen::Matrix4d Tbh;
Eigen::Matrix4d LInvTkf;
Eigen::Matrix4d LInvTtk;
Eigen::Matrix4d LInvTht;
Eigen::Matrix4d LInvTbh;
Eigen::Matrix4d LTkf;
Eigen::Matrix4d LTtk;
Eigen::Matrix4d LTht;
Eigen::Matrix4d LTbh;
Eigen::Matrix4d RightRearFoot2Base;
Eigen::Matrix4d Base2RightRearFoot;
Eigen::Matrix4d LeftRearFoot2Base;
Eigen::Matrix4d Base2LeftRearFoot;
Eigen::Matrix4d TEST;
Eigen::Vector4d InitialPos;
Eigen::Vector4d posBody;
Eigen::Vector4d posRFB;
Eigen::Vector4d posLFB;
Eigen::Vector4d prevPos;
Eigen::Vector4d startPos;
Eigen::Vector4d startPosFB;
Eigen::Vector4d bias;
Eigen::Vector4d worldBody;
bool RightFirst = 1;
bool LeftFirst = 1;



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
//    smem->GetPosX = MPCcontroller.cmpcSolver.p[0];
    smem->DesPosX = (worldBody[0]);
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

void raisimSimulation(double timeD)
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

///Right Rear LEg
        Tbh << 1, 0, 0, -0.175,
            0, std::cos(robot.robot->getGeneralizedCoordinate()[13]), -std::sin(robot.robot->getGeneralizedCoordinate()[13]), -0.055,
            0, std::sin(robot.robot->getGeneralizedCoordinate()[13]), std::cos(robot.robot->getGeneralizedCoordinate()[13]), 0,
            0, 0, 0, 1;

        Tht << std::cos(robot.robot->getGeneralizedCoordinate()[14]), 0, std::sin(robot.robot->getGeneralizedCoordinate()[14]), -0.0705,
            0, 1, 0, -0.030496,
            -std::sin(robot.robot->getGeneralizedCoordinate()[14]), 0, std::cos(robot.robot->getGeneralizedCoordinate()[14]), 0,
            0, 0, 0, 1;

        Ttk << std::cos(robot.robot->getGeneralizedCoordinate()[15]), 0, std::sin(robot.robot->getGeneralizedCoordinate()[15]), 0,
            0, 1, 0, -0.077,
            -std::sin(robot.robot->getGeneralizedCoordinate()[15]), 0, std::cos(robot.robot->getGeneralizedCoordinate()[15]), -0.23,
            0, 0, 0, 1;

        alpha = -(robot.robot->getGeneralizedCoordinate()[15]+robot.robot->getGeneralizedCoordinate()[14]);
        Tkf << std::cos(alpha), 0, std::sin(alpha), 0,
            0, 1, 0, 0,
            -std::sin(alpha), 0, std::cos(alpha), -0.23,
            0, 0, 0, 1;

        InvTbh = Tbh.inverse();
        InvTht = Tht.inverse();
        InvTtk = Ttk.inverse();
        InvTkf = Tkf.inverse();

        RightRearFoot2Base = InvTkf*InvTtk*InvTht*InvTbh;
        Base2RightRearFoot = Tbh*Tht*Ttk*Tkf;

///Left Rear Leg
        LTbh<< 1, 0, 0, -0.175,
            0, std::cos(robot.robot->getGeneralizedCoordinate()[16]), -std::sin(robot.robot->getGeneralizedCoordinate()[16]), 0.055,
            0, std::sin(robot.robot->getGeneralizedCoordinate()[16]), std::cos(robot.robot->getGeneralizedCoordinate()[16]), 0,
            0, 0, 0, 1;
        LTht<< std::cos(robot.robot->getGeneralizedCoordinate()[17]), 0, std::sin(robot.robot->getGeneralizedCoordinate()[17]), -0.0705,
            0, 1, 0, 0.030496,
            -std::sin(robot.robot->getGeneralizedCoordinate()[17]), 0, std::cos(robot.robot->getGeneralizedCoordinate()[17]), 0,
            0, 0, 0, 1;
        LTtk<< std::cos(robot.robot->getGeneralizedCoordinate()[18]), 0, std::sin(robot.robot->getGeneralizedCoordinate()[18]), 0,
            0, 1, 0, 0.077,
            -std::sin(robot.robot->getGeneralizedCoordinate()[18]), 0, std::cos(robot.robot->getGeneralizedCoordinate()[18]), -0.23,
            0, 0, 0, 1;

        beta = -(robot.robot->getGeneralizedCoordinate()[18]+robot.robot->getGeneralizedCoordinate()[17]);
        LTkf << std::cos(beta), 0, std::sin(beta), 0,
            0, 1, 0, 0,
            -std::sin(alpha), 0, std::cos(alpha), -0.23,
            0, 0, 0, 1;

        LInvTbh = LTbh.inverse();
        LInvTht = LTht.inverse();
        LInvTtk = LTtk.inverse();
        LInvTkf = LTkf.inverse();

        Base2LeftRearFoot = LTbh*LTht*LTtk*LTkf;
        LeftRearFoot2Base = LInvTkf*LInvTtk*LInvTht*LInvTbh;

        if(*(MPCcontroller.GetMpcTable()+1)==1) //swing 0, contact 1
        {
            if(RightFirst == 1)
            {
                RightFirst = 0;
                LeftFirst = 1;
                startPos = worldBody;
                prevPos = RightRearFoot2Base*startPos;
            }
            posBody[0] = RightRearFoot2Base(0,3);
            worldBody[0] = worldBody[0] + posBody[0]-prevPos[0];
//            std::cout << (posBody - prevPos)[0] << " "<< posBody[0] <<" "<< RightRearFoot2Base(0,3) <<std::endl;

            prevPos[0] = posBody[0];
        }
        else
        {
            if(LeftFirst == 1)
            {
                LeftFirst = 0;
                RightFirst = 1;
                startPos = worldBody;
                prevPos[0] = LeftRearFoot2Base(0,3);
            }
            posBody[0] = LeftRearFoot2Base(0,3);
            worldBody[0] = worldBody[0] + (posBody[0] - prevPos[0]);
            std::cout << posBody[0] - prevPos[0] << " "<< posBody[0] <<" "<< LeftRearFoot2Base(0,3) <<std::endl;
            prevPos[0] = posBody[0];
        }

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

        raisimSimulation(timediff_us(&TIME_NEXT, &TIME_NOW)* 0.000001);

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


    InitialPos[0]=robot.robot->getGeneralizedCoordinate()[0];
    InitialPos[1]=robot.robot->getGeneralizedCoordinate()[1];
    InitialPos[2]=robot.robot->getGeneralizedCoordinate()[2];
    InitialPos[3] = 1;
    worldBody = InitialPos;
    bias << 0.25,0,0,0;
    posBody = InitialPos;

//    world.addArticulatedSystem("\\home\\cha\\git\\repository-group\\raisimLib\\camel-code-raisim-cpp\\rsc\\obstacle.urdf");
    raisim::RaisimServer server(&world);
    server.launchServer(8080);

    int thread_id_timeChecker = generate_rt_thread(thread_simulation, rt_simulation_thread, "simulation_thread", 7, 99,
                                                   NULL);

    w.show();
    return a.exec();

}

//todo 4열 계산 실수 (전부 다)

//            InvTtk(0,0) = Ttk(0,0), InvTtk(0,1) = Ttk(1,0), InvTtk(0,2) = Ttk(2,0), InvTtk(0,3) = 0.23*std::sin(robot.robot->getGeneralizedCoordinate()[9]),
//            InvTtk(1,0) = Ttk(0,1), InvTtk(1,1) = Ttk(1,1), InvTtk(1,2) = Ttk(2,1), InvTtk(1,3) = 0.077,
//            InvTtk(2,0) = Ttk(0,2), InvTtk(2,1) = Ttk(1,2), InvTtk(2,2) = Ttk(2,2), InvTtk(2,3) = 0.23*std::cos(robot.robot->getGeneralizedCoordinate()[9]),
//            InvTtk(3,0) = 0, InvTtk(3,1) = 0, InvTtk(3,2) = 0, InvTtk(3,3) = 1;


//            InvTht(0,0) = Tht(0,0), InvTht(0,1) = Tht(1,0), InvTht(0,2) = Tht(2,0), InvTht(0,3) = -0.075*std::cos(robot.robot->getGeneralizedCoordinate()[8]),
//            InvTht(1,0) = Tht(0,1), InvTht(1,1) = Tht(1,1), InvTht(1,2) = Tht(2,1), InvTht(1,3) = 0.030496,
//            InvTht(2,0) = Tht(0,2), InvTht(2,1) = Tht(1,2), InvTht(2,2) = Tht(2,2), InvTht(2,3) = 0.075*std::sin(robot.robot->getGeneralizedCoordinate()[8]),
//            InvTht(3,0) = 0, InvTht(3,1) = 0, InvTht(3,2) = 0, InvTht(3,3) = 1;

//            InvTbh(0,0) = Tbh(0,0), InvTbh(0,1) = Tbh(1,0), InvTbh(0,2) = Tbh(2,0), InvTbh(0,3) = -0.175,
//            InvTbh(1,0) = Tbh(0,1), InvTbh(1,1) = Tbh(1,1), InvTbh(1,2) = Tbh(2,1), InvTbh(1,3) = 0.055*std::cos(robot.robot->getGeneralizedCoordinate()[7]),
//            InvTbh(2,0) = Tbh(0,2), InvTbh(2,1) = Tbh(1,2), InvTbh(2,2) = Tbh(2,2), InvTbh(2,3) = 0.055*std::sin(robot.robot->getGeneralizedCoordinate()[7]),
//            InvTbh(3,0) = 0, InvTbh(3,1) = 0, InvTbh(3,2) = 0, InvTbh(3,3) = 1;