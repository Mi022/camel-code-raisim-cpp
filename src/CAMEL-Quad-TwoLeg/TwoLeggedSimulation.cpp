//
// Created by jaehoon on 22. 3. 31.. dkswogns46@gmail.com
//

#include "TwoLeggedSimulation.h"
#include "include/SimulationUI/simulationMainwindow.h"
#include "include/RT/rb_utils.h"
#include <QApplication>
#include <cmath>

extern MainWindow *MainUI;
bool timeChecker = false;

void thread1task(raisim::World *world, TwoLeggedRobot *robot, TwoLeggedPDController *controller, double simulationDuration) {
    double dT = world->getTimeStep();
    double oneCycleSimTime = 0;
    int divider = ceil(simulationDuration / dT / 200);
    int i = 0;
//    auto begin = std::chrono::high_resolution_clock::now();
//    auto end = std::chrono::high_resolution_clock::now();
    while (true) {
        if (timeChecker) {
            if ((MainUI->button1) && (oneCycleSimTime < simulationDuration)) {
                // control robot and data plot thread
//                if(i == 0){begin = std::chrono::high_resolution_clock::now();}
                oneCycleSimTime = i * dT;
                controller->doControl();
                world->integrate();
                if (i % divider == 0) {
//                    std::cout<<"data_idx : "<<MainUI->data_idx<<std::endl;
                    MainUI->data_x[MainUI->data_idx] = world->getWorldTime();
                    MainUI->data_y1[MainUI->data_idx] = robot->getQ()[0];
                    MainUI->data_y2[MainUI->data_idx] = robot->getQD()[0];
                    MainUI->data_idx += 1;
                }
                i++;
                timeChecker = false;
            } else if (oneCycleSimTime >= simulationDuration) {
//                end = std::chrono::high_resolution_clock::now();
//                auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
//                std::cout <<" Time measured: "<< elapsed.count() * 1e-9 <<"seconds" <<std::endl;
                MainUI->button1 = false;
                i = 0;
                oneCycleSimTime = 0;
                MainUI->plotWidget1();
                MainUI->plotWidget2();
                MainUI->data_idx = 0;
            }
//
//            if ((world->getWorldTime()) > 5.0) {
//                Eigen::VectorXd positionTarget(12), velocityTarget(12);
//                positionTarget << 0.0, 0.5, -1.2, 0.0, 0.5, -1.2, 0.0, 0.5, -1.2, 0.0, 0.5, -1.2;
//                velocityTarget.setZero();
//                controller->updateJointTrajectory(positionTarget, velocityTarget);
            }
        }
    }
}

void thread2task() {
    while (true) {
        std::this_thread::sleep_for(std::chrono::microseconds(500));
        timeChecker = true;
    }
}

int main(int argc, char *argv[]) {
    std::string urdfPath = "\\home\\user\\raisimLib\\camel-code-raisim-cpp\\rsc\\camel_two_leg\\camel_two_leg.urdf";
    std::string name = "cuteTwoLeg";
    raisim::World world;
    double simulationDuration = 3.0;
    TwoLeggedSimulation sim = TwoLeggedSimulation(&world, 0.001);
    TwoLeggedRobot robotTwoLegged = TwoLeggedRobot(&world, urdfPath, name);
    TwoLeggedPDController PDcontroller = TwoLeggedPDController(&robotTwoLegged);

    raisim::RaisimServer server(&world);
    server.launchServer(8080);
    server.integrateWorldThreadSafe();

    QApplication a(argc, argv);
    MainWindow w;
    std::thread thread1(thread1task, &world, &robotTwoLegged, &PDcontroller, simulationDuration);
    std::thread thread2(thread2task);
    w.show();

    return a.exec();
}
