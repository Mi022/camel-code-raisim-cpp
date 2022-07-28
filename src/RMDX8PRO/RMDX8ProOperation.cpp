//
// Created by jaehoon on 22. 4. 13..
//

#include "RMDX8ProOperation.h"
#include "mainwindow.h"
#include <QApplication>
#include <thread>

double deg2rad = 3.141592 / 180.0;
double rad2deg = 180.0 / 3.141592;

void thread1task(bool *button1Pressed, bool *button2Pressed, bool *button3Pressed, bool *button4Pressed,
                 bool *button5Pressed, bool *button6Pressed) {
    std::string canName_temp = "can8";
    std::string bitRate = "1000000";
    char *canName = "can8";
    CanMotorX8ProV2 canX8pro(canName, canName_temp, bitRate);

    std::string urdfPath = "\\home\\jaehoon\\raisimLib\\camel-code-raisim-cpp\\rsc\\camel_RMDX8_pro.urdf";
    std::string name = "RMDX8Pro";
    raisim::World world;
    RMDX8ProOperation realRobot = RMDX8ProOperation(&world, 30);
    RMDX8ProRobot rmdX8 = RMDX8ProRobot(&world, urdfPath, name, &canX8pro);
    RMDX8ProPDContorller PDcontroller = RMDX8ProPDContorller(&rmdX8);
    raisim::RaisimServer server(&world);

    while (true) {
        usleep(1000);
        PDcontroller.updateState();

        if (*button1Pressed) {
            // CAN initialize
            if (canX8pro.getSock() < 0) {
                std::cout << "Failed to creat CAN" << std::endl;
                break;
            }
            std::cout << "Success to initialize CAN communication" << std::endl;
            *button1Pressed = false;
        }

        if (*button2Pressed) {
            // Raisim initialize

            server.launchServer(8080);
            sleep(5);
            *button2Pressed = false;
            std::cout << "Success to initialize Raisim" << std::endl;
        }

        if (*button3Pressed) {
            // Motor On
            rmdX8.motorOn(0x141);
            *button3Pressed = false;
        }

        if (*button4Pressed) {
            // Motor Off
            rmdX8.motorOff(0x141);
            *button4Pressed = false;
        }

        if (*button5Pressed) {
            // Start Control
            int iteration = 0;
            while (true) {
                iteration++;
                PDcontroller.doControl();
                if ((PDcontroller.isTerminateCondition()) || (iteration > 1000)) {
                    rmdX8.motorOff(0x141);
                    break;
                }
                usleep(1000);
                std::cout << "current position : " << PDcontroller.position << std::endl;
            }
            *button5Pressed = false;
        }

        if (*button6Pressed) {
            // Generate new trajectory
            PDcontroller.setTrajectory(3.141592 * 1.0, 0.0);
            *button6Pressed = false;
        }
    }
}

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    MainWindow w;
    std::thread thread1(thread1task, &w.button1, &w.button2, &w.button3, &w.button4, &w.button5, &w.button6);
    w.show();

    return a.exec();
}


