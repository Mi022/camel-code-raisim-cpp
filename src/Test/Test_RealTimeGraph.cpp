//
// Created by jaehoon on 22. 7. 11.
//
#include "simulationMainwindow.h"
#include <QApplication>
extern MainWindow *MainUI;

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}