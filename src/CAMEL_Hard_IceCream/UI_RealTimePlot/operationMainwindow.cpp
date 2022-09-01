//
// Created by jaehoon on 22. 7. 19.
//

#include "operationMainwindow.h"
#include "ui_operationMainwindow.h"
#include "../IceCreamSharedMemory.h"
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
    ui->widget->graph(0)->setName("torque");
    ui->widget->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget->addGraph();
    ui->widget->graph(1)->setName("desired position");
    ui->widget->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    ui->widget_2->legend->setVisible(true);
    ui->widget_2->legend->setFont(QFont("Helvetica", 9));
    ui->widget_2->addGraph();
    ui->widget_2->graph(0)->setName("measured acc0");
    ui->widget_2->graph(0)->setPen(QPen(QColor(0, 0, 255)));
//    ui->widget_2->addGraph();
//    ui->widget_2->graph(1)->setName("calculated acc0");
//    ui->widget_2->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_2->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    ui->widget_3->legend->setVisible(true);
    ui->widget_3->legend->setFont(QFont("Helvetica", 9));
    ui->widget_3->addGraph();
    ui->widget_3->graph(0)->setName("measured acc1");
    ui->widget_3->graph(0)->setPen(QPen(QColor(0, 0, 255)));
//    ui->widget_3->addGraph();
//    ui->widget_3->graph(1)->setName("calculated acc1");
//    ui->widget_3->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_3->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
    dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::on_pushButton_clicked() {
    std::cout << "'Run' button is clicked" << std::endl;
    button1 = !button1;
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
        lastPointKey = key;
    }

// calculate frames per second:
    static double lastFpsKey;
    static int frameCount;
    ++frameCount;
    if (key-lastFpsKey > 2) // average fps over 2 seconds
    {
        ui->statusBar->showMessage(
                QString("%1 FPS, Total Data points: %2")
                        .arg(frameCount/(key-lastFpsKey), 0, 'f', 0)
                        .arg(ui->widget->graph(0)->data()->size()+ui->widget->graph(1)->data()->size())
                , 0);
        lastFpsKey = key;
        frameCount = 0;
    }
}

void MainWindow::plotWidget1() {
    if (sharedMemory->plotW1B < yMinWidget1) { yMinWidget1 = sharedMemory->plotW1B; }
    if (sharedMemory->plotW1B > yMaxWidget1) { yMaxWidget1 = sharedMemory->plotW1B; }
    if (sharedMemory->plotW1R < yMinWidget1) { yMinWidget1 = sharedMemory->plotW1R; }
    if (sharedMemory->plotW1R > yMaxWidget1) { yMaxWidget1 = sharedMemory->plotW1R; }
    ui->widget->graph(0)->addData(sharedMemory->simTime, sharedMemory->plotW1B);
    ui->widget->graph(1)->addData(sharedMemory->simTime, sharedMemory->plotW1R);

    // set axes ranges, so we see all data:
    ui->widget->xAxis->setRange(0.0, sharedMemory->simTime + 0.001);
    ui->widget->yAxis->setRange(yMinWidget1 - 0.001, yMaxWidget1 + 0.001);
    ui->widget->replot();
}

void MainWindow::plotWidget2() {
    if (sharedMemory->plotW2B < yMinWidget2) { yMinWidget2 = sharedMemory->plotW2B; }
    if (sharedMemory->plotW2B > yMaxWidget2) { yMaxWidget2 = sharedMemory->plotW2B; }
//    if (sharedMemory->plotW2R < yMinWidget2) { yMinWidget2 = sharedMemory->plotW2R; }
//    if (sharedMemory->plotW2R > yMaxWidget2) { yMaxWidget2 = sharedMemory->plotW2R; }
    ui->widget_2->graph(0)->addData(sharedMemory->plotW2R, sharedMemory->plotW2B);
//    ui->widget_2->graph(1)->addData(sharedMemory->simTime, sharedMemory->plotW2R);

    // set axes ranges, so we see all data:
    ui->widget_2->xAxis->setRange(0.0, sharedMemory->plotW2R + 0.001);
    ui->widget_2->yAxis->setRange(yMinWidget2 - 0.001, yMaxWidget2 + 0.001);
    ui->widget_2->replot();
}

void MainWindow::plotWidget3() {
    if (sharedMemory->plotW3B < yMinWidget3) { yMinWidget3 = sharedMemory->plotW3B; }
    if (sharedMemory->plotW3B > yMaxWidget3) { yMaxWidget3 = sharedMemory->plotW3B; }
//    if (sharedMemory->plotW3R < yMinWidget3) { yMinWidget3 = sharedMemory->plotW3R; }
//    if (sharedMemory->plotW3R > yMaxWidget3) { yMaxWidget3 = sharedMemory->plotW3R; }
    ui->widget_3->graph(0)->addData(sharedMemory->plotW3R, sharedMemory->plotW3B);
//    ui->widget_3->graph(1)->addData(sharedMemory->simTime, sharedMemory->plotW3R);

    // set axes ranges, so we see all data:
    ui->widget_3->xAxis->setRange(0.0, sharedMemory->plotW3R + 0.001);
    ui->widget_3->yAxis->setRange(yMinWidget3 - 0.1, yMaxWidget3 + 0.1);
    ui->widget_3->replot();
}

