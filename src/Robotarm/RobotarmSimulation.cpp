//
// Created by jy on 22. 7. 4.
//
#ifndef RAISIM_SIMPLEPENDULUMSIMULATION_H
#define RAISIM_SIMPLEPENDULUMSIMULATION_H

#include "include/CAMEL/Simulation.h"
#include "include/SimulationUI/simulationMainwindow.h"
#include "include/RT/rb_utils.h"
#include <QApplication>

extern MainWindow *MainUI;
pthread_t thread_simulation;
std::string urdfPath = "\\home\\jy\\raisimLib\\camel-code-raisim-cpp\\rsc\\ur_five.urdf";
std::string name = "UR_five";
raisim::World world;


int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    MainWindow w;
    raisim::RaisimServer server(&world);
    server.launchServer(8080);
    w.show();

    return a.exec();
}

#endif