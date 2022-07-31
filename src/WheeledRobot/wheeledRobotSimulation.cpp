//
// Created by ljm on 22. 7. 6.
//

#include "wheeledRobotSimulation.h"
#include "wheeledRobotSimulationWindow.h"
#include "wheeledRobotXBOX.h"
#include "include/RT/rb_utils.h"
#include <QApplication>
#include <cmath>

extern MainWindow *MainUI;
pthread_t thread_simulation;

wheeledRobotJoyStick joystick = wheeledRobotJoyStick();

std::string urdfPath = "\\home\\ljm\\raisimLib\\camel-code-raisim-cpp\\rsc\\camel_wheeledRobot.urdf";
std::string name = "wheeledRobot";
raisim::World world;
raisim::RaisimServer server(&world);

double simulationDuration = 2.0;
double dT = world.getTimeStep();
wheeledRobotSimulation sim = wheeledRobotSimulation(&world, 0.002);
wheeledRobot robot = wheeledRobot(&world, urdfPath, name);
wheeledRobotController controller = wheeledRobotController(&robot);

double oneCycleSimTime = 0;
int divider = ceil(simulationDuration / dT / 200);
int iteration = 0;

/*double* x, y, z;
&x = 0.0;
&y = 0.0;
Eigen::Vector3d pos = {&x, y, z};
Eigen::Vector3d lookAt = {1, 2, 3};*/

void raisimSimulation() {
    if(joystick.joyAvailable) {
        joystick.joyRead();

        if ((joystick.joy_button[11] || joystick.joy_button[12] || joystick.joy_button[13] || joystick.joy_button[14]
             || joystick.joy_axis[0]/10000 || joystick.joy_axis[1]/10000)
            || (MainUI->button1)||(MainUI->button2)||(MainUI->button3)||(MainUI->button4)
                                                                        && (oneCycleSimTime < simulationDuration)) {
            oneCycleSimTime = iteration * dT;

            if(joystick.joy_button[0]) {
                controller.accelerate();
            }
                /*else if(joystick.joy_axis[5]>-32767) {
                    controller.setVel((joystick.joy_axis[5] / 10000) + 4);
                }*/
            else if(joystick.joy_axis[1]/10000) {
                controller.setVel(joystick.joy_axis[1] / 10000);
            }
            else if(joystick.joy_axis[0]/10000) {
                controller.setVel(-abs(joystick.joy_axis[0]) / 10000);
            }
            else {
                controller.setStop();
            }

            if(joystick.joy_button[13] || (joystick.joy_axis[1]/10000)<0) {
                if(joystick.joy_button[11] || (joystick.joy_axis[0]/10000)<0)  {
                    controller.setLeft();
                }
                else if(joystick.joy_button[12] || (joystick.joy_axis[0]/10000)>0) {
                    controller.setRight();
                }
                else {
                    controller.setForward();
                }
            }

            if(joystick.joy_button[11] || (joystick.joy_axis[0]/10000)<0)  {
                controller.setLeft();
            }

            if(joystick.joy_button[12] || (joystick.joy_axis[0]/10000)>0) {
                controller.setRight();
            }

            if(joystick.joy_button[14] || (joystick.joy_axis[1]/10000)>0) {
                if(joystick.joy_button[11] || (joystick.joy_axis[0]/10000)<0)  {
                    controller.setLeft();
                }
                else if(joystick.joy_button[12] || (joystick.joy_axis[0]/10000)>0) {
                    controller.setRight();
                }
                else {
                    controller.setBack();
                }
            }
            iteration++;
        }
        else if (oneCycleSimTime >= simulationDuration) {
            /*
            MainUI->button1 = false;
            MainUI->button2 = false;
            MainUI->button3 = false;
            MainUI->button4 = false;
             */

            iteration = 0;
            oneCycleSimTime = 0;

            std::cout << "torque : " << controller.torque[6] << " " << controller.torque[7] << std::endl;
            std::cout << "velocity : " << controller.velocity[8] << " " << controller.velocity[9] << std::endl;

            MainUI->plotWidget1();
            MainUI->plotWidget2();
            MainUI->data_idx = 0;
        }
        else {
            controller.setStop();
        }

        if(joystick.joy_button[6]) {
            server.killServer();
            exit(0);
        }
    }
    else {
        if (((MainUI->button1)||(MainUI->button2)||(MainUI->button3)||(MainUI->button4)||(MainUI->button5)) && (oneCycleSimTime < simulationDuration)) {
            oneCycleSimTime = iteration * dT;

            if(MainUI->button1) {
                controller.setForward();
            }

            if(MainUI->button2)  {
                controller.setLeft();
            }

            if(MainUI->button3) {
                controller.setRight();
            }

            if(MainUI->button4) {
                controller.setBack();
            }

            if(MainUI->button5) {
                controller.setStop();
            }

            iteration++;
        }
        else if (oneCycleSimTime >= simulationDuration) {
            MainUI->button1 = false;
            MainUI->button2 = false;
            MainUI->button3 = false;
            MainUI->button4 = false;
            MainUI->button5 = false;

            iteration = 0;
            oneCycleSimTime = 0;

            std::cout << "torque : " << controller.torque[6] << " " << controller.torque[7] << std::endl;
            std::cout << "velocity : " << controller.velocity[8] << " " << controller.velocity[9] << std::endl;

            MainUI->plotWidget1();
            MainUI->plotWidget2();
            MainUI->data_idx = 0;
        }
    }

    controller.doControl();
    world.integrate();

    /*if(joystick.joy_axis[3]/10000) {
        x++; y++; z++;
        server.setCameraPositionAndLookAt(pos, lookAt);
    }*/



}

void *rt_simulation_thread(void *arg) {
    std::cout << "entered #rt_time_checker_thread" << std::endl;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    const long PERIOD_US = long(dT * 1e6);

    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    std::cout << "bf #while" << std::endl;
    std::cout << "control freq : " << 1 / double(PERIOD_US) * 1e6 << std::endl;

    while (true) {
        clock_gettime(CLOCK_REALTIME, &TIME_NOW); //현재 시간 구함
        timespec_add_us(&TIME_NEXT, PERIOD_US);   //목표 시간 구함

        raisimSimulation();

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
        if (timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0) {
            std::cerr << "RT Deadline Miss, Time Checker thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001
                      << " ms" << std::endl;
        }
    }
}

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    MainWindow w;
    sim.setGroundProperty("wheat");

    server.launchServer(8080);
    server.focusOn(robot.robot);

    int thread_id_timeChecker = generate_rt_thread(thread_simulation, rt_simulation_thread, "simulation_thread", 0, 99,
                                                   NULL);
    w.show();

    return a.exec();
}