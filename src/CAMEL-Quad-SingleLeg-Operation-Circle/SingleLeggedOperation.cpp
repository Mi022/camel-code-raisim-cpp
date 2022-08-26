//
// Created by jaehoon on 22. 5. 25.
//

#include "UI/mainwindow.h"
#include "include/RT/rb_utils.h"
#include "SingleLeggedOperation.h"
#include "SingleLeggedSharedMemoryOperation.h"
#include <QApplication>
#include <random>

double deg2rad = 3.141592 / 180.0;
double rad2deg = 180.0 / 3.141592;
double currentTime = 0.0;
double dT = 0.005;

pthread_t thread_operation;
pSHM sharedMemory;

bool isReady = false;
bool *buttonCANInitPressed;
bool *buttonRaisimInitPressed;
bool *buttonMotorOnPressed;
bool *buttonMotorOffPressed;
bool *buttonStartControlPressed;
bool *buttonStopControlPressed;
bool *buttonGenCubicTrajPressed;
bool *buttonGenSinTrajPressed;
bool *buttonJumpPressed;
bool *buttonZeroingPressed;

std::string canName_temp = "can0";
std::string bitRate = "1000000";
char *canName = "can0";
SingleLegCAN can(canName, canName_temp, bitRate);
int motorHip = 0x141;
int motorKnee = 0x142;
double intr = 1.0;



std::string urdfPath = "\\home\\camel\\raisimLib\\camel-code-raisim-cpp\\rsc\\camel_single_leg_right\\camel_single_leg.urdf";
std::string name = "singleLeg";
raisim::World world;
SingleLeggedOperation realRobot = SingleLeggedOperation(&world, 250);
SingleLeggedRobotOperation singleLeg = SingleLeggedRobotOperation(&world, urdfPath, name, &can, dT);
SingleLeggedPDControllerOperation controller = SingleLeggedPDControllerOperation(&singleLeg, &currentTime, dT);
//SingleLeggedIDControllerOperation controller = SingleLeggedIDControllerOperation(&singleLeg, &currentTime, dT);
raisim::RaisimServer server(&world);

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_int_distribution<int> dis(0, 99);
double randomGoalPosition;

void updateSHM(){
    sharedMemory->time = currentTime;
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

void operationCode(){
    singleLeg.visualize();
    updateSHM();
    if(isReady)
    {
        can.setTorque(motorHip, 0.0);
        can.setTorque(motorKnee, 0.0);
        controller.updateState();
//        singleLeg.getQ();
//        singleLeg.getQD();
    }
    if (*buttonCANInitPressed) {
        // CAN initialize
        if (can.getSock() < 0) {
            std::cout << "Failed to creat CAN" << std::endl;
            return;
        }
        std::cout << "Success to initialize CAN communication" << std::endl;
        *buttonCANInitPressed = false;
    }

    if (*buttonRaisimInitPressed) {
        // Raisim initialize
        server.launchServer(8080);
        *buttonRaisimInitPressed = false;
        std::cout << "Success to initialize Raisim" << std::endl;
    }

    if (*buttonMotorOnPressed) {
        // Motor On
        can.turnOnMotor(motorKnee);
        can.turnOnMotor(motorHip);
        can.setTorque(motorHip, 0.0);
        can.setTorque(motorKnee, 0.0);
        isReady = true;
        *buttonMotorOnPressed = false;
    }

    if (*buttonMotorOffPressed) {
        // Motor Off
        can.turnOffMotor(motorKnee);
        can.turnOffMotor(motorHip);
        isReady = false;
        *buttonMotorOffPressed = false;
    }

    if (*buttonStartControlPressed) {
        controller.doControl();
    }

    if (*buttonStopControlPressed) {
        can.turnOffMotor(motorKnee);
        can.turnOffMotor(motorHip);
        *buttonZeroingPressed = false;
        *buttonStartControlPressed = false;
        *buttonStopControlPressed = false;
    }

    if (*buttonGenCubicTrajPressed){
        randomGoalPosition = double(dis(gen)) / 100.0 * 0.15 + 0.23;
        intr = -intr;
        double goalPos = 0.30 + 0.06 * intr;
//        controller.updateCubicTrajectory(randomGoalPosition, 2.0);
        controller.updateCubicTrajectory(goalPos, 2.0);

        *buttonGenCubicTrajPressed = false;
    }

    if (*buttonGenSinTrajPressed){
        *buttonGenSinTrajPressed = false;
    }

    if (*buttonJumpPressed){
        *buttonJumpPressed = false;
    }

    if (*buttonZeroingPressed){
        std::cout << "zeroing start" << std::endl;
        controller.zeroing();
        isReady = false;
        *buttonStartControlPressed = true;
        *buttonZeroingPressed = false;
    }
}

void *rt_operation_thread(void *arg) {
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

        currentTime += dT;
        operationCode();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL); //목표시간까지 기다림 (현재시간이 이미 오바되어 있으면 바로 넘어갈 듯)
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) {  // 현재시간이 목표시간 보다 오바되면 경고 띄우기
            std::cout << "RT Deadline Miss, Operation thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001
                      << " ms" << std::endl;
        }
    }
}

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    MainWindow w;
    sharedMemory = (pSHM) malloc(sizeof(SHM));
    buttonCANInitPressed = &w.buttonCANInit;
    buttonRaisimInitPressed = &w.buttonRaisimInit;
    buttonMotorOnPressed = &w.buttonMotorOn;
    buttonMotorOffPressed = &w.buttonMotorOff;
    buttonStartControlPressed = &w.buttonStartControl;
    buttonStopControlPressed = &w.buttonStopControl;
    buttonGenCubicTrajPressed = &w.buttonGenerateCubicTrajectory;
    buttonGenSinTrajPressed = &w.buttonGenerateSinTrajectory;
    buttonJumpPressed = &w.buttonJump;
    buttonZeroingPressed = &w.buttonZeroing;

    int thread_id_operation = generate_rt_thread(thread_operation, rt_operation_thread, "operation_thread", 0, 49, NULL);
    std::cout<<"test"<<std::endl;
    w.show();

    return a.exec();
}


