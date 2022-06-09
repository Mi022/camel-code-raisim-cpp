//
// Created by jaehoon on 22. 5. 25.
//

#include "SingleLeggedOperation.h"
#include "mainwindow.h"
#include "include/RT/rb_utils.h"
#include "include/Sensor/LoadCell.h"
#include <QApplication>
#include <thread>
#include <random>

double deg2rad = 3.141592 / 180.0;
double rad2deg = 180.0 / 3.141592;
double currentTime = 0.0;
pthread_t thread_operation;
pthread_t thread_loadcell;

bool isrealTimePlot = false;
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
int motorKnee = 0x141;
int motorHip = 0x143;
double intr = 1.0;

LoadCell sensorLoadcell;
double sensoredForce = 0.0;

std::string urdfPath = "\\home\\jaehoon\\raisimLib\\camel-code-raisim-cpp\\rsc\\camel_single_leg\\camel_single_leg.urdf";
std::string name = "singleLeg";
raisim::World world;
SingleLeggedOperation realRobot = SingleLeggedOperation(&world, 250);
SingleLeggedRobotOperation singleLeg = SingleLeggedRobotOperation(&world, urdfPath, name, &can);
SingleLeggedPDControllerOperation controller = SingleLeggedPDControllerOperation(&singleLeg, &currentTime);
raisim::RaisimServer server(&world);

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_int_distribution<int> dis(0, 99);
double randomGoalPosition;

void operationCode(){
    singleLeg.visualize();

    if(isrealTimePlot)
    {
        can.readEncoder(motorHip);
        can.readEncoder(motorKnee);
        singleLeg.visualize();
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
        singleLeg.visualize();
        isrealTimePlot = true;
        *buttonMotorOnPressed = false;
    }

    if (*buttonMotorOffPressed) {
        // Motor Off
        can.turnOffMotor(motorKnee);
        can.turnOffMotor(motorHip);
        *buttonMotorOffPressed = false;
    }

    if (*buttonStartControlPressed) {
//             Start Control
        controller.doControl();
        std::cout<<"===================================================="<<std::endl;
        std::cout<<"current time: "<<currentTime<<std::endl;
        std::cout<<"current position : "<<controller.position[1]<<" "<<controller.position[2]<<std::endl;
        std::cout<<"desired position : "<<controller.desiredJointPosition[0] <<" "<<controller.desiredJointPosition[1]<<std::endl;
        std::cout<<"current velocity : "<<controller.velocity[1]<<" "<<controller.velocity[2]<<std::endl;
        std::cout<<"desired velocity : "<<controller.desiredJointVelocity[0] <<" "<<controller.desiredJointVelocity[1]<<std::endl;
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
        *buttonStartControlPressed = true;
        *buttonZeroingPressed = false;
    }
}

void *rt_operation_thread(void *arg) {
    std::cout << "entered #rt_time_checker_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(0.005 * 1e6); // 200Hz 짜리 쓰레드

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    std::cout << "bf #while" << std::endl;
    std::cout << "control freq : " << 1 / double(PERIOD_US) * 1e6 << std::endl;
    while (true) {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW); //현재 시간 구함
        timespec_add_us(&TIME_NEXT, PERIOD_US);   //목표 시간 구함

        currentTime += 0.005;
        operationCode();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL); //목표시간까지 기다림 (현재시간이 이미 오바되어 있으면 바로 넘어갈 듯)
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) {  // 현재시간이 목표시간 보다 오바되면 경고 띄우기
            std::cout << "RT Deadline Miss, Operation thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001
                      << " ms" << std::endl;
        }
    }
}

void *rt_loadcell_thread(void *arg){
    while(true)
    {
        sensorLoadcell.readData();
        sensoredForce = sensorLoadcell.getSensoredForce();
//        std::cout<<"Sensored force[N] : "<<sensoredForce<<std::endl;
    }
}

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    MainWindow w;
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

    int thread_id_operation = generate_rt_thread(thread_operation, rt_operation_thread, "operation_thread", 0, 99, NULL);
    int thread_id_sensorLoadcell = generate_rt_thread(thread_loadcell, rt_loadcell_thread, "sensor_loadcell_thread", 1, 98, NULL);
    w.show();

    return a.exec();
}


