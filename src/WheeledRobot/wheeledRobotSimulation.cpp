//
// Created by ljm on 22. 7. 6.
//

#include "wheeledRobotSimulation.h"
#include "wheeledRobotSimulationWindow.h"
#include "wheeledRobotXBOX.h"
#include "include/RT/rb_utils.h"
#include <QApplication>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

#define JOY_DEV "/dev/input/js0"

int joy_fd=-1;
int num_of_axis=0;
int num_of_buttons=0;
char name_of_joystick[80];
std::vector<char> joy_button;
std::vector<int> joy_axis;

js_event js;

extern MainWindow *MainUI;
pthread_t thread_simulation;

std::string urdfPath = "\\home\\ljm\\raisimLib\\camel-code-raisim-cpp\\rsc\\camel_wheeledRobot.urdf";
std::string name = "wheeledRobot";
raisim::World world;

double simulationDuration = 2.0;
double dT = world.getTimeStep();
wheeledRobotSimulation sim = wheeledRobotSimulation(&world, 0.002);
wheeledRobot robot = wheeledRobot(&world, urdfPath, name);
wheeledRobotController controller = wheeledRobotController(&robot);

double oneCycleSimTime = 0;
int divider = ceil(simulationDuration / dT / 200);
int iteration = 0;

void joySetup() {
    if((joy_fd = open(JOY_DEV,O_RDONLY)) < 0)
    {
        std::cerr<<"Failed to open "<<JOY_DEV<<std::endl;
    }

    ioctl(joy_fd, JSIOCGAXES, &num_of_axis);
    ioctl(joy_fd, JSIOCGBUTTONS, &num_of_buttons);
    ioctl(joy_fd, JSIOCGNAME(80), &name_of_joystick);

    joy_button.resize(num_of_buttons,0);
    joy_axis.resize(num_of_axis,0);

    std::cout<<"Joystick: "<<name_of_joystick<<std::endl
             <<"  axis: "<<num_of_axis<<std::endl
             <<"  buttons: "<<num_of_buttons<<std::endl;

    fcntl(joy_fd, F_SETFL, O_NONBLOCK);
}

void joyRead() {
    read(joy_fd, &js, sizeof(js_event));

    switch (js.type & ~JS_EVENT_INIT)
    {
        case JS_EVENT_AXIS:
            if((int)js.number>=joy_axis.size())  {std::cerr<<"err:"<<(int)js.number<<std::endl;}
            joy_axis[(int)js.number]= js.value;
            break;
        case JS_EVENT_BUTTON:
            if((int)js.number>=joy_button.size())  {std::cerr<<"err:"<<(int)js.number<<std::endl;}
            joy_button[(int)js.number]= js.value;
            break;
    }
}

void raisimSimulation() {
    joyRead();
    int button = 0;
    //if (((MainUI->button1)||(MainUI->button2)||(MainUI->button3)||(MainUI->button4)) && (oneCycleSimTime < simulationDuration))
    if ((joy_button[11] || joy_button[12] || joy_button[13] || joy_button[14] || joy_button[0] || joy_axis[0]/10000 || joy_axis[1]/10000) && (oneCycleSimTime < simulationDuration)) {
        oneCycleSimTime = iteration * dT;

        if(joy_button[0]) {
            controller.accelerate();
        }
        else {
            controller.setVel(-joy_axis[1]/10000);
        }

        if(joy_button[13] || (joy_axis[1]/10000)<0) {
            if(joy_button[11] || (joy_axis[0]/10000)<0)  {
                controller.setLeft();
            }
            else if(joy_button[12] || (joy_axis[0]/10000)>0) {
                controller.setRight();
            }
            else {
                controller.setForward();
            }
        }

        if(joy_button[11] || (joy_axis[0]/10000)<0)  {
            controller.setLeft();
        }

        if(joy_button[12] || (joy_axis[0]/10000)>0) {
            controller.setRight();
        }

        if(joy_button[14] || (joy_axis[1]/10000)>0) {
            if(joy_button[11] || (joy_axis[0]/10000)<0)  {
                controller.setLeft();
            }
            else if(joy_button[12] || (joy_axis[0]/10000)>0) {
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

        /*
        MainUI->plotWidget1();
        MainUI->plotWidget2();
        MainUI->data_idx = 0;
         */
    }
    else {
        controller.setStop();
    }
    controller.doControl();
    world.integrate();

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
            std::cout << "RT Deadline Miss, Time Checker thread : " << timediff_us(&TIME_NEXT, &TIME_NOW) * 0.001
                      << " ms" << std::endl;
        }
    }
}

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    MainWindow w;
    sim.setGroundProperty("wheat");
    raisim::RaisimServer server(&world);
    server.launchServer(8080);

    joySetup();

    int thread_id_timeChecker = generate_rt_thread(thread_simulation, rt_simulation_thread, "simulation_thread", 0, 99,
                                                   NULL);
    w.show();

    return a.exec();
}