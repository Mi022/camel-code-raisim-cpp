#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "../SingleLeggedSharedMemoryOperation.h"
#include <iostream>

extern pSHM sharedMemory;

MainWindow::MainWindow(QWidget *parent)
        : QMainWindow(parent)
        , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->widget_1->legend->setVisible(true);
    ui->widget_1->legend->setFont(QFont("Helvetica", 9));
    ui->widget_1->addGraph();
    ui->widget_1->graph(0)->setName("position_z");
    ui->widget_1->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_1->addGraph();
    ui->widget_1->graph(1)->setName("desired position_z");
    ui->widget_1->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_1->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    ui->widget_2->legend->setVisible(true);
    ui->widget_2->legend->setFont(QFont("Helvetica", 9));
    ui->widget_2->addGraph();
    ui->widget_2->graph(0)->setName("velocity_z");
    ui->widget_2->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_2->addGraph();
    ui->widget_2->graph(1)->setName("desired velocity_z");
    ui->widget_2->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_2->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    ui->widget_3->legend->setVisible(true);
    ui->widget_3->legend->setFont(QFont("Helvetica", 9));
    ui->widget_3->addGraph();
    ui->widget_3->graph(0)->setName("hip_position");
    ui->widget_3->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_3->addGraph();
    ui->widget_3->graph(1)->setName("knee_position");
    ui->widget_3->graph(1)->setPen(QPen(QColor(0, 255, 0)));
    ui->widget_3->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    ui->widget_4->legend->setVisible(true);
    ui->widget_4->legend->setFont(QFont("Helvetica", 9));
    ui->widget_4->addGraph();
    ui->widget_4->graph(0)->setName("hip_velocity");
    ui->widget_4->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_4->addGraph();
    ui->widget_4->graph(1)->setName("knee_velocity");
    ui->widget_4->graph(1)->setPen(QPen(QColor(0, 255, 0)));
    ui->widget_4->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    ui->widget_5->legend->setVisible(true);
    ui->widget_5->legend->setFont(QFont("Helvetica", 9));
    ui->widget_5->addGraph();
    ui->widget_5->graph(0)->setName("hip_torque");
    ui->widget_5->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_5->addGraph();
    ui->widget_5->graph(1)->setName("knee_torque");
    ui->widget_5->graph(1)->setPen(QPen(QColor(0, 255, 0)));
    ui->widget_5->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);

    ui->widget_6->legend->setVisible(true);
    ui->widget_6->legend->setFont(QFont("Helvetica", 9));
    ui->widget_6->addGraph();
    ui->widget_6->graph(0)->setName("GRF");
    ui->widget_6->graph(0)->setPen(QPen(QColor(0, 0, 255)));
//    ui->widget_6->addGraph();
//    ui->widget_6->graph(1)->setName("torque");
//    ui->widget_6->graph(1)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_6->setInteractions(QCP::iRangeZoom | QCP::iRangeDrag);
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
    dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

MainWindow::~MainWindow()
{
    delete ui;
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
        plotWidget5();
        plotWidget6();
        lastPointKey = key;
    }

// calculate frames per second:
    static double lastFpsKey;
    static int frameCount;
    ++frameCount;
    if (key-lastFpsKey > 2) // average fps over 2 seconds
    {
//        ui->statusBar->showMessage(
//                QString("%1 FPS, Total Data points: %2")
//                        .arg(frameCount/(key-lastFpsKey), 0, 'f', 0)
//                        .arg(ui->widget->graph(0)->data()->size()+ui->widget->graph(1)->data()->size())
//                , 0);
        lastFpsKey = key;
        frameCount = 0;
    }
}

void MainWindow::plotWidget1() {
    if (sharedMemory->position_z < yMinWidget1) { yMinWidget1 = sharedMemory->position_z; }
    if (sharedMemory->position_z > yMaxWidget1) { yMaxWidget1 = sharedMemory->position_z; }
    if (sharedMemory->desiredPosition_z < yMinWidget1) { yMinWidget1 = sharedMemory->desiredPosition_z; }
    if (sharedMemory->desiredPosition_z > yMaxWidget1) { yMaxWidget1 = sharedMemory->desiredPosition_z; }
    ui->widget_1->graph(0)->addData(sharedMemory->time, sharedMemory->position_z);
    ui->widget_1->graph(1)->addData(sharedMemory->time, sharedMemory->desiredPosition_z);

    // set axes ranges, so we see all data:
    ui->widget_1->xAxis->setRange(sharedMemory->time - intervalTime, sharedMemory->time + 0.001);
    ui->widget_1->yAxis->setRange(yMinWidget1 - 0.001, yMaxWidget1 + 0.001);
    ui->widget_1->replot();
}

void MainWindow::plotWidget2() {
    if (sharedMemory->velocity_z < yMinWidget2) { yMinWidget2 = sharedMemory->velocity_z; }
    if (sharedMemory->velocity_z > yMaxWidget2) { yMaxWidget2 = sharedMemory->velocity_z; }
    if (sharedMemory->desiredVelocity_z < yMinWidget2) { yMinWidget2 = sharedMemory->desiredVelocity_z; }
    if (sharedMemory->desiredVelocity_z > yMaxWidget2) { yMaxWidget2 = sharedMemory->desiredVelocity_z; }
    ui->widget_2->graph(0)->addData(sharedMemory->time, sharedMemory->velocity_z);
    ui->widget_2->graph(1)->addData(sharedMemory->time, sharedMemory->desiredVelocity_z);

    // set axes ranges, so we see all data:
    ui->widget_2->xAxis->setRange(sharedMemory->time - intervalTime, sharedMemory->time + 0.001);
    ui->widget_2->yAxis->setRange(yMinWidget2 - 0.01, yMaxWidget2 + 0.01);
    ui->widget_2->replot();
}

void MainWindow::plotWidget3() {
    if (sharedMemory->jointPosition[0] < yMinWidget3) { yMinWidget3 = sharedMemory->jointPosition[0]; }
    if (sharedMemory->jointPosition[0] > yMaxWidget3) { yMaxWidget3 = sharedMemory->jointPosition[0]; }
    if (sharedMemory->jointPosition[1] < yMinWidget3) { yMinWidget3 = sharedMemory->jointPosition[1]; }
    if (sharedMemory->jointPosition[1] > yMaxWidget3) { yMaxWidget3 = sharedMemory->jointPosition[1]; }

    ui->widget_3->graph(0)->addData(sharedMemory->time, sharedMemory->jointPosition[0]);
    ui->widget_3->graph(1)->addData(sharedMemory->time, sharedMemory->jointPosition[1]);

    // set axes ranges, so we see all data:
    ui->widget_3->xAxis->setRange(sharedMemory->time - intervalTime, sharedMemory->time + 0.001);
    ui->widget_3->yAxis->setRange(yMinWidget3 - 0.1, yMaxWidget3 + 0.1);
    ui->widget_3->replot();
}

void MainWindow::plotWidget4() {
    if (sharedMemory->jointVelocity[0] < yMinWidget4) { yMinWidget4 = sharedMemory->jointVelocity[0]; }
    if (sharedMemory->jointVelocity[0] > yMaxWidget4) { yMaxWidget4 = sharedMemory->jointVelocity[0]; }
    if (sharedMemory->jointVelocity[1] < yMinWidget4) { yMinWidget4 = sharedMemory->jointVelocity[1]; }
    if (sharedMemory->jointVelocity[1] > yMaxWidget4) { yMaxWidget4 = sharedMemory->jointVelocity[1]; }
    ui->widget_4->graph(0)->addData(sharedMemory->time, sharedMemory->jointVelocity[0]);
    ui->widget_4->graph(1)->addData(sharedMemory->time, sharedMemory->jointVelocity[1]);

    // set axes ranges, so we see all data:
    ui->widget_4->xAxis->setRange(sharedMemory->time - intervalTime, sharedMemory->time + 0.001);
    ui->widget_4->yAxis->setRange(yMinWidget4 - 0.01, yMaxWidget4 + 0.01);
    ui->widget_4->replot();
}

void MainWindow::plotWidget5() {
    if (sharedMemory->jointTorque[0] < yMinWidget5) { yMinWidget5 = sharedMemory->jointTorque[0]; }
    if (sharedMemory->jointTorque[0] > yMaxWidget5) { yMaxWidget5 = sharedMemory->jointTorque[0]; }
    if (sharedMemory->jointTorque[1] < yMinWidget5) { yMinWidget5 = sharedMemory->jointTorque[1]; }
    if (sharedMemory->jointTorque[1] > yMaxWidget5) { yMaxWidget5 = sharedMemory->jointTorque[1]; }
    ui->widget_5->graph(0)->addData(sharedMemory->time, sharedMemory->jointTorque[0]);
    ui->widget_5->graph(1)->addData(sharedMemory->time, sharedMemory->jointTorque[1]);

    // set axes ranges, so we see all data:
    ui->widget_5->xAxis->setRange(sharedMemory->time - intervalTime, sharedMemory->time + 0.001);
    ui->widget_5->yAxis->setRange(yMinWidget5 - 0.01, yMaxWidget5 + 0.01);
    ui->widget_5->replot();
}

void MainWindow::plotWidget6() {
    if (sharedMemory->GRF < yMinWidget6) { yMinWidget6 = sharedMemory->GRF; }
    if (sharedMemory->GRF > yMaxWidget6) { yMaxWidget6 = sharedMemory->GRF; }
    ui->widget_6->graph(0)->addData(sharedMemory->time, sharedMemory->GRF);

    // set axes ranges, so we see all data:
    ui->widget_6->xAxis->setRange(sharedMemory->time - intervalTime, sharedMemory->time + 0.001);
    ui->widget_6->yAxis->setRange(yMinWidget6 - 0.01, yMaxWidget6 + 0.01);
    ui->widget_6->replot();
}

void MainWindow::on_pushButton_1_clicked()
{
    // CAN Init
    if (buttonCANInit) { buttonCANInit = false; }
    else { buttonCANInit = true; }
    std::cout<<"CAN Init button is clicked."<<std::endl;
}

void MainWindow::on_pushButton_2_clicked()
{
    // Raisim Init
    if (buttonRaisimInit) { buttonRaisimInit = false; }
    else { buttonRaisimInit = true; }
    std::cout<<"Raisim Init button is clicked."<<std::endl;
}

void MainWindow::on_pushButton_3_clicked()
{
    // Motor On
    if (buttonMotorOn) { buttonMotorOn = false; }
    else { buttonMotorOn = true; }
    std::cout<<"Motor On button is clicked."<<std::endl;
}

void MainWindow::on_pushButton_4_clicked()
{
    // Motor Off
    if (buttonMotorOff) { buttonMotorOff = false; }
    else { buttonMotorOff = true; }
    std::cout<<"Motor Off button is clicked."<<std::endl;
}

void MainWindow::on_pushButton_5_clicked()
{
    // Start Control
    if (buttonStartControl) { buttonStartControl = false; }
    else { buttonStartControl = true; }
    std::cout<<"Start Control button is clicked."<<std::endl;
}

void MainWindow::on_pushButton_6_clicked()
{
    // Stop Control
    if (buttonStopControl) { buttonStopControl = false; }
    else { buttonStopControl = true; }
    std::cout<<"Stop Control button is clicked."<<std::endl;
}

void MainWindow::on_pushButton_7_clicked()
{
    // Generate new cubic trajectory
    if (buttonGenerateCubicTrajectory) { buttonGenerateCubicTrajectory = false; }
    else { buttonGenerateCubicTrajectory = true; }
    std::cout<<"Generate Cubic Trajectory button is clicked."<<std::endl;
}

void MainWindow::on_pushButton_8_clicked()
{
    // Generate new sin trajectory
    if (buttonGenerateSinTrajectory) { buttonGenerateSinTrajectory = false; }
    else { buttonGenerateSinTrajectory = true; }
    std::cout<<"Generate Sin Trajectory button is clicked."<<std::endl;
}

void MainWindow::on_pushButton_9_clicked()
{
    // Jump
    if (buttonJump) { buttonJump = false; }
    else { buttonJump = true; }
    std::cout<<"Jump button is clicked."<<std::endl;
}

void MainWindow::on_pushButton_10_clicked()
{
    if (buttonZeroing) { buttonZeroing = false; }
    else { buttonZeroing = true; }
    std::cout<<"Zeroing button is clicked."<<std::endl;
}
