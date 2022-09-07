//
// Created by jaehoon on 22. 7. 11.
//
#include "src/Test/Qt_RealTimeGraph/simulationMainwindow.h"
#include <QApplication>
extern MainWindow *MainUI;

int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}