//
// Created by jaehoon on 22. 4. 26.
//

#include "simulationMainwindow.h"
#include "ui_simulationMainwindow.h"
#include "../SingleLeggedSharedMemory.h"
#include <iostream>

extern pSHM sharedMemory;

MainWindow *MainUI;

MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::MainWindow) {
    MainUI = this;
    ui->setupUi(this);

    ui->widget->legend->setVisible(true);
    ui->widget->legend->setFont(QFont("Helvetica", 9));
    ui->widget->addGraph();
    ui->widget->graph(0)->setName("position_z");
    ui->widget->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget->addGraph();
    ui->widget->graph(1)->setName("desired position_z");
    ui->widget->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft | Qt::AlignTop);
    ui->widget->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    ui->widget_2->legend->setVisible(true);
    ui->widget_2->legend->setFont(QFont("Helvetica", 9));
    ui->widget_2->addGraph();
    ui->widget_2->graph(0)->setName("hip velocity");
    ui->widget_2->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_2->addGraph();
    ui->widget_2->graph(1)->setName("desired hip velocity");
    ui->widget_2->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_2->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft | Qt::AlignTop);
    ui->widget_2->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    ui->widget_3->legend->setVisible(true);
    ui->widget_3->legend->setFont(QFont("Helvetica", 9));
    ui->widget_3->addGraph();
    ui->widget_3->graph(0)->setName("knee velocity");
    ui->widget_3->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_3->addGraph();
    ui->widget_3->graph(1)->setName("desired knee velocity");
    ui->widget_3->graph(1)->setPen(QPen(QColor(0, 255, 0)));
    ui->widget_3->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft | Qt::AlignTop);
    ui->widget_3->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
    dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::realtimeDataSlot() {
    static QTime time(QTime::currentTime());
    double key = time.elapsed()/1000.0;
    int fps = 120;
    static double lastPointKey = 0;
    if ((key-lastPointKey > double(1/fps))&&(!isSimulationEnd)) // at most add point every 10 ms
    {
        plotWidget1();
        plotWidget2();
        plotWidget3();
        lastPointKey = key;
    }
}

void MainWindow::on_pushButton_clicked() {
    std::cout << "'Run' button is clicked" << std::endl;
    isSimulationEnd = false;
    if (button1) { button1 = false; }
    else { button1 = true; }
}

void MainWindow::plotWidget1() {
    if (sharedMemory->position_z < yMinWidget1) { yMinWidget1 = sharedMemory->position_z; }
    if (sharedMemory->position_z > yMaxWidget1) { yMaxWidget1 = sharedMemory->position_z; }
    if (sharedMemory->desiredPosition_z < yMinWidget1) { yMinWidget1 = sharedMemory->desiredPosition_z; }
    if (sharedMemory->desiredPosition_z > yMaxWidget1) { yMaxWidget1 = sharedMemory->desiredPosition_z; }
    ui->widget->graph(0)->addData(sharedMemory->time, sharedMemory->position_z);
    ui->widget->graph(1)->addData(sharedMemory->time, sharedMemory->desiredPosition_z);

    // set axes ranges, so we see all data:
    ui->widget->xAxis->setRange(sharedMemory->time - intervalTime, sharedMemory->time + 0.001);
    ui->widget->yAxis->setRange(yMinWidget1 - 0.001, yMaxWidget1 + 0.001);
    ui->widget->replot();
}

void MainWindow::plotWidget2() {
//    if (sharedMemory->velocity_z < yMinWidget2) { yMinWidget2 = sharedMemory->velocity_z; }
//    if (sharedMemory->velocity_z > yMaxWidget2) { yMaxWidget2 = sharedMemory->velocity_z; }
//    if (sharedMemory->desiredVelocity_z < yMinWidget2) { yMinWidget2 = sharedMemory->desiredVelocity_z; }
//    if (sharedMemory->desiredVelocity_z > yMaxWidget2) { yMaxWidget2 = sharedMemory->desiredVelocity_z; }
//    ui->widget_2->graph(0)->addData(sharedMemory->time, sharedMemory->velocity_z);
//    ui->widget_2->graph(1)->addData(sharedMemory->time, sharedMemory->desiredVelocity_z);

    if (sharedMemory->jointVelocity[0] < yMinWidget2) { yMinWidget2 = sharedMemory->jointVelocity[0]; }
    if (sharedMemory->jointVelocity[0] > yMaxWidget2) { yMaxWidget2 = sharedMemory->jointVelocity[0]; }
    if (sharedMemory->desiredJointVelocity[0] < yMinWidget2) { yMinWidget2 = sharedMemory->desiredJointVelocity[0]; }
    if (sharedMemory->desiredJointVelocity[0] > yMaxWidget2) { yMaxWidget2 = sharedMemory->desiredJointVelocity[0]; }
    ui->widget_2->graph(0)->addData(sharedMemory->time, sharedMemory->jointVelocity[0]);
    ui->widget_2->graph(1)->addData(sharedMemory->time, sharedMemory->desiredJointVelocity[0]);

    // set axes ranges, so we see all data:
    ui->widget_2->xAxis->setRange(sharedMemory->time - intervalTime, sharedMemory->time + 0.001);
    ui->widget_2->yAxis->setRange(yMinWidget2 - 0.001, yMaxWidget2 + 0.001);
    ui->widget_2->replot();
}

void MainWindow::plotWidget3() {
//    if (sharedMemory->jointTorque[0] < yMinWidget3) { yMinWidget3 = sharedMemory->jointTorque[0]; }
//    if (sharedMemory->jointTorque[0] > yMaxWidget3) { yMaxWidget3 = sharedMemory->jointTorque[0]; }
//    if (sharedMemory->jointTorque[1] < yMinWidget3) { yMinWidget3 = sharedMemory->jointTorque[1]; }
//    if (sharedMemory->jointTorque[1] > yMaxWidget3) { yMaxWidget3 = sharedMemory->jointTorque[1]; }
//    ui->widget_3->graph(0)->addData(sharedMemory->time, sharedMemory->jointTorque[0]);
//    ui->widget_3->graph(1)->addData(sharedMemory->time, sharedMemory->jointTorque[1]);

    if (sharedMemory->jointVelocity[1] < yMinWidget3) { yMinWidget3 = sharedMemory->jointVelocity[1]; }
    if (sharedMemory->jointVelocity[1] > yMaxWidget3) { yMaxWidget3 = sharedMemory->jointVelocity[1]; }
    if (sharedMemory->desiredJointVelocity[1] < yMinWidget3) { yMinWidget3 = sharedMemory->desiredJointVelocity[1]; }
    if (sharedMemory->desiredJointVelocity[1] > yMaxWidget3) { yMaxWidget3 = sharedMemory->desiredJointVelocity[1]; }
    ui->widget_3->graph(0)->addData(sharedMemory->time, sharedMemory->jointVelocity[1]);
    ui->widget_3->graph(1)->addData(sharedMemory->time, sharedMemory->desiredJointVelocity[1]);

    // set axes ranges, so we see all data:
    ui->widget_3->xAxis->setRange(sharedMemory->time - intervalTime, sharedMemory->time + 0.001);
    ui->widget_3->yAxis->setRange(yMinWidget3 - 0.01, yMaxWidget3 + 0.01);
    ui->widget_3->replot();
}

