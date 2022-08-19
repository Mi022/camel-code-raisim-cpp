//
// Created by jaehoon on 22. 7. 19.
//

#include "operationMainwindow.h"
#include "ui_operationMainwindow.h"
#include "../JHdogSharedMemory.h"
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
    ui->widget->graph(0)->setName("Height");
    ui->widget->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    ui->widget_2->legend->setVisible(true);
    ui->widget_2->legend->setFont(QFont("Helvetica", 9));
    ui->widget_2->addGraph();
    ui->widget_2->graph(0)->setName("hip pitch position");
    ui->widget_2->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_2->addGraph();
    ui->widget_2->graph(1)->setName("desired hip pitch position");
    ui->widget_2->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_2->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    ui->widget_3->legend->setVisible(true);
    ui->widget_3->legend->setFont(QFont("Helvetica", 9));
    ui->widget_3->addGraph();
    ui->widget_3->graph(0)->setName("hip pitch velocity");
    ui->widget_3->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_3->addGraph();
    ui->widget_3->graph(1)->setName("desired hip pitch velocity");
    ui->widget_3->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_3->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    ui->widget_4->legend->setVisible(true);
    ui->widget_4->legend->setFont(QFont("Helvetica", 9));
    ui->widget_4->addGraph();
    ui->widget_4->graph(0)->setName("hip pitch torque");
    ui->widget_4->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_4->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));

    dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::on_pushButton_clicked() {
    std::cout << "'Run' button is clicked" << std::endl;
    if (button1) { button1 = false; }
    else { button1 = true; }
}


void MainWindow::realtimeDataSlot() {
    static QTime time(QTime::currentTime());
    double key = time.elapsed()/1000.0;
    int fps = 120;
    static double lastPointKey = 0;
    if (key-lastPointKey > double(1/fps)) // at most add point every 10 ms
    {
        plotWidget1();
        plotWidget2();
        plotWidget3();
        plotWidget4();
        lastPointKey = key;
    }
//
// calculate frames per second:
    static double lastFpsKey;
    static int frameCount;
    ++frameCount;
    if (key-lastFpsKey > 2) // average fps over 2 seconds
    {
        std::cout<<"Hi"<<std::endl;
        ui->statusBar->showMessage(
                QString("%1 FPS, Total Data points: %2")
                        .arg(frameCount/(key-lastFpsKey), 0, 'f', 0)
                        .arg(ui->widget_2->graph(0)->data()->size()+ui->widget_2->graph(1)->data()->size())
                , 0);

        lastFpsKey = key;
        frameCount = 0;
    }
}

//Height
void MainWindow::plotWidget1() {
    if (sharedMemory->jointPosition[0] < yMinWidget1) { yMinWidget1 = sharedMemory->jointPosition[0]; }
    if (sharedMemory->jointPosition[0] > yMaxWidget1) { yMaxWidget1 = sharedMemory->jointPosition[0]; }
    ui->widget->graph(0)->addData(sharedMemory->simTime, sharedMemory->jointPosition[0]);

    // set axes ranges, so we see all data:
    ui->widget->xAxis->setRange(0.0, sharedMemory->simTime + 0.001);
    ui->widget->yAxis->setRange(yMinWidget1 - 0.001, yMaxWidget1 + 0.001);
    ui->widget->replot();
}

//hip pitch position
void MainWindow::plotWidget2() {
    if (sharedMemory->jointPosition[1] < yMinWidget2) { yMinWidget2 = sharedMemory->jointPosition[1]; }
    if (sharedMemory->jointPosition[1] > yMaxWidget2) { yMaxWidget2 = sharedMemory->jointPosition[1]; }
    if (sharedMemory->desiredJointPosition[1] < yMinWidget2) { yMinWidget2 = sharedMemory->desiredJointPosition[1]; }
    if (sharedMemory->desiredJointPosition[1] > yMaxWidget2) { yMaxWidget2 = sharedMemory->desiredJointPosition[1]; }
    ui->widget_2->graph(0)->addData(sharedMemory->simTime, sharedMemory->jointPosition[1]);
    ui->widget_2->graph(1)->addData(sharedMemory->simTime, sharedMemory->desiredJointPosition[1]);

    // set axes ranges, so we see all data:
    ui->widget_2->xAxis->setRange(0.0, sharedMemory->simTime + 0.001);
    ui->widget_2->yAxis->setRange(yMinWidget2 - 0.001, yMaxWidget2 + 0.001);
    ui->widget_2->replot();
}

//hip pitch velocity
void MainWindow::plotWidget3() {
    if (sharedMemory->jointVelocity[1] < yMinWidget3) { yMinWidget3 = sharedMemory->jointVelocity[1]; }
    if (sharedMemory->jointVelocity[1] > yMaxWidget3) { yMaxWidget3 = sharedMemory->jointVelocity[1]; }
    if (sharedMemory->desiredJointVelocity[1] < yMinWidget3) { yMinWidget3 = sharedMemory->desiredJointVelocity[1]; }
    if (sharedMemory->desiredJointVelocity[1] > yMaxWidget3) { yMaxWidget3 = sharedMemory->desiredJointVelocity[1]; }
    ui->widget_3->graph(0)->addData(sharedMemory->simTime, sharedMemory->jointVelocity[1]);
    ui->widget_3->graph(1)->addData(sharedMemory->simTime, sharedMemory->desiredJointVelocity[1]);

    // set axes ranges, so we see all data:
    ui->widget_3->xAxis->setRange(0.0, sharedMemory->simTime + 0.001);
    ui->widget_3->yAxis->setRange(yMinWidget3 - 0.001, yMaxWidget3 + 0.001);
    ui->widget_3->replot();
}

//hip pitch torque
void MainWindow::plotWidget4() {
    if (sharedMemory->jointTorque[1] < yMinWidget4) { yMinWidget4 = sharedMemory->jointTorque[1]; }
    if (sharedMemory->jointTorque[1] > yMaxWidget4) { yMaxWidget4 = sharedMemory->jointTorque[1]; }
    ui->widget_4->graph(0)->addData(sharedMemory->simTime, sharedMemory->jointTorque[1]);

//  set axes ranges, so we see all data
    ui->widget_4->xAxis->setRange(0.0, sharedMemory->simTime + 0.001);
    ui->widget_4->yAxis->setRange(yMinWidget4 - 0.001, yMaxWidget4 + 0.001);
    ui->widget_4->replot();
}