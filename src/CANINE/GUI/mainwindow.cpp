#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "SharedMemory.h"
#include <iostream>

extern pSHM smem;
MainWindow *MainUI;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    MainUI = this;
    ui->setupUi(this);

    ui->widget_1->legend->setVisible(true);
    ui->widget_1->legend->setFont(QFont("Helvetica", 9));
    ui->widget_1->addGraph();
    ui->widget_1->graph(0)->setName("position X");
    ui->widget_1->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_1->addGraph();
    ui->widget_1->graph(1)->setName("desired position X");
    ui->widget_1->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_1->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    ui->widget_2->legend->setVisible(true);
    ui->widget_2->legend->setFont(QFont("Helvetica", 9));
    ui->widget_2->addGraph();
    ui->widget_2->graph(0)->setName("position Y");
    ui->widget_2->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_2->addGraph();
    ui->widget_2->graph(1)->setName("desired position Y");
    ui->widget_2->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_2->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    ui->widget_3->legend->setVisible(true);
    ui->widget_3->legend->setFont(QFont("Helvetica", 9));
    ui->widget_3->addGraph();
    ui->widget_3->graph(0)->setName("position Z");
    ui->widget_3->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_3->addGraph();
    ui->widget_3->graph(1)->setName("desired position Z");
    ui->widget_3->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_3->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    ui->widget_4->legend->setVisible(true);
    ui->widget_4->legend->setFont(QFont("Helvetica", 9));
    ui->widget_4->addGraph();
    ui->widget_4->graph(0)->setName("rotation X");
    ui->widget_4->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_4->addGraph();
    ui->widget_4->graph(1)->setName("desired rotation X");
    ui->widget_4->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_4->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    ui->widget_5->legend->setVisible(true);
    ui->widget_5->legend->setFont(QFont("Helvetica", 9));
    ui->widget_5->addGraph();
    ui->widget_5->graph(0)->setName("rotation Y");
    ui->widget_5->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_5->addGraph();
    ui->widget_5->graph(1)->setName("desired rotation Y");
    ui->widget_5->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_5->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    ui->widget_6->legend->setVisible(true);
    ui->widget_6->legend->setFont(QFont("Helvetica", 9));
    ui->widget_6->addGraph();
    ui->widget_6->graph(0)->setName("rotation Z");
    ui->widget_6->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_6->addGraph();
    ui->widget_6->graph(1)->setName("desired rotation Z");
    ui->widget_6->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_6->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    ui->widget_7->legend->setVisible(true);
    ui->widget_7->legend->setFont(QFont("Helvetica", 9));
    ui->widget_7->addGraph();
    ui->widget_7->graph(0)->setName("FR");
    ui->widget_7->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_7->addGraph();
    ui->widget_7->graph(1)->setName("FL");
    ui->widget_7->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_7->addGraph();
    ui->widget_7->graph(2)->setName("RR");
    ui->widget_7->graph(2)->setPen(QPen(QColor(0, 255, 0)));
    ui->widget_7->addGraph();
    ui->widget_7->graph(3)->setName("RL");
    ui->widget_7->graph(3)->setPen(QPen(QColor(255, 255, 0)));
    ui->widget_7->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    ui->widget_8->legend->setVisible(true);
    ui->widget_8->legend->setFont(QFont("Helvetica", 9));
    ui->widget_8->addGraph();
    ui->widget_8->graph(0)->setName("FR");
    ui->widget_8->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_8->addGraph();
    ui->widget_8->graph(1)->setName("FL");
    ui->widget_8->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_8->addGraph();
    ui->widget_8->graph(2)->setName("RR");
    ui->widget_8->graph(2)->setPen(QPen(QColor(0, 255, 0)));
    ui->widget_8->addGraph();
    ui->widget_8->graph(3)->setName("RL");
    ui->widget_8->graph(3)->setPen(QPen(QColor(255, 255, 0)));
    ui->widget_8->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    ui->widget_9->legend->setVisible(true);
    ui->widget_9->legend->setFont(QFont("Helvetica", 9));
    ui->widget_9->addGraph();
    ui->widget_9->graph(0)->setName("FR");
    ui->widget_9->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_9->addGraph();
    ui->widget_9->graph(1)->setName("FL");
    ui->widget_9->graph(1)->setPen(QPen(QColor(255, 0, 0)));
    ui->widget_9->addGraph();
    ui->widget_9->graph(2)->setName("RR");
    ui->widget_9->graph(2)->setPen(QPen(QColor(0, 255, 0)));
    ui->widget_9->addGraph();
    ui->widget_9->graph(3)->setName("RL");
    ui->widget_9->graph(3)->setPen(QPen(QColor(255, 255, 0)));
    ui->widget_9->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
    dataTimer.start(0);
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_pushButton_clicked()
{
    std::cout << "'Run' button is clicked" << std::endl;
    if (button1) { button1 = false; }
    else { button1 = true; }
}

void MainWindow::on_gaitBox_currentIndexChanged(int index)
{
    gaitIdx = index;
    gaitChanged = 1;
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

        plotWidget7();
        plotWidget8();
        plotWidget9();

        lastPointKey = key;
    }
}

void MainWindow::plotWidget1() {
    if (smem->GetPosX < yMinWidget1) { yMinWidget1 = smem->GetPosX; }
    if (smem->GetPosX > yMaxWidget1) { yMaxWidget1 = smem->GetPosX; }
    if (smem->DesPosX < yMinWidget1) { yMinWidget1 = smem->DesPosX; }
    if (smem->DesPosX > yMaxWidget1) { yMaxWidget1 = smem->DesPosX; }
    ui->widget_1->graph(0)->addData(smem->simTime, smem->GetPosX);
    ui->widget_1->graph(1)->addData(smem->simTime, smem->DesPosX);

    // set axes ranges, so we see all data:
    ui->widget_1->xAxis->setRange(0.0, smem->simTime + 0.001);
    ui->widget_1->yAxis->setRange(yMinWidget1 - 0.001, yMaxWidget1 + 0.001);
    ui->widget_1->replot();
}

void MainWindow::plotWidget2() {
    if (smem->GetPosY < yMinWidget2) { yMinWidget2 = smem->GetPosY; }
    if (smem->GetPosY > yMaxWidget2) { yMaxWidget2 = smem->GetPosY; }
    if (smem->DesPosY < yMinWidget2) { yMinWidget2 = smem->DesPosY; }
    if (smem->DesPosY > yMaxWidget2) { yMaxWidget2 = smem->DesPosY; }
    ui->widget_2->graph(0)->addData(smem->simTime, smem->GetPosY);
    ui->widget_2->graph(1)->addData(smem->simTime, smem->DesPosY);

    // set axes ranges, so we see all data:
    ui->widget_2->xAxis->setRange(0.0, smem->simTime + 0.001);
    ui->widget_2->yAxis->setRange(yMinWidget2 - 0.001, yMaxWidget2 + 0.001);
    ui->widget_2->replot();
}

void MainWindow::plotWidget3() {
    if (smem->GetPosZ < yMinWidget3) { yMinWidget3 = smem->GetPosZ; }
    if (smem->GetPosZ > yMaxWidget3) { yMaxWidget3 = smem->GetPosZ; }
    if (smem->DesPosZ < yMinWidget3) { yMinWidget3 = smem->DesPosZ; }
    if (smem->DesPosZ > yMaxWidget3) { yMaxWidget3 = smem->DesPosZ; }
    ui->widget_3->graph(0)->addData(smem->simTime, smem->GetPosZ);
    ui->widget_3->graph(1)->addData(smem->simTime, smem->DesPosZ);

    // set axes ranges, so we see all data:
    ui->widget_3->xAxis->setRange(0.0, smem->simTime + 0.001);
    ui->widget_3->yAxis->setRange(yMinWidget3 - 0.001, yMaxWidget3 + 0.001);
    ui->widget_3->replot();
}

void MainWindow::plotWidget4() {
    if (smem->GetRotX < yMinWidget4) { yMinWidget4 = smem->GetRotX; }
    if (smem->GetRotX > yMaxWidget4) { yMaxWidget4 = smem->GetRotX; }
    if (smem->DesRotX < yMinWidget4) { yMinWidget4 = smem->DesRotX; }
    if (smem->DesRotX > yMaxWidget4) { yMaxWidget4 = smem->DesRotX; }
    ui->widget_4->graph(0)->addData(smem->simTime, smem->GetRotX);
    ui->widget_4->graph(1)->addData(smem->simTime, smem->DesRotX);

    // set axes ranges, so we see all data:
    ui->widget_4->xAxis->setRange(0.0, smem->simTime + 0.001);
    ui->widget_4->yAxis->setRange(yMinWidget4 - 0.001, yMaxWidget4 + 0.001);
    ui->widget_4->replot();
}

void MainWindow::plotWidget5() {
    if (smem->GetRotY < yMinWidget5) { yMinWidget5 = smem->GetRotY; }
    if (smem->GetRotY > yMaxWidget5) { yMaxWidget5 = smem->GetRotY; }
    if (smem->DesRotY < yMinWidget5) { yMinWidget5 = smem->DesRotY; }
    if (smem->DesRotY > yMaxWidget5) { yMaxWidget5 = smem->DesRotY; }
    ui->widget_5->graph(0)->addData(smem->simTime, smem->GetRotY);
    ui->widget_5->graph(1)->addData(smem->simTime, smem->DesRotY);

    // set axes ranges, so we see all data:
    ui->widget_5->xAxis->setRange(0.0, smem->simTime + 0.001);
    ui->widget_5->yAxis->setRange(yMinWidget5 - 0.001, yMaxWidget5 + 0.001);
    ui->widget_5->replot();
}

void MainWindow::plotWidget6() {
    if (smem->GetRotZ < yMinWidget6) { yMinWidget6 = smem->GetRotZ; }
    if (smem->GetRotZ > yMaxWidget6) { yMaxWidget6 = smem->GetRotZ; }
    if (smem->DesRotZ < yMinWidget6) { yMinWidget6 = smem->DesRotZ; }
    if (smem->DesRotZ > yMaxWidget6) { yMaxWidget6 = smem->DesRotZ; }
    ui->widget_6->graph(0)->addData(smem->simTime, smem->GetRotZ);
    ui->widget_6->graph(1)->addData(smem->simTime, smem->DesRotZ);

    // set axes ranges, so we see all data:
    ui->widget_6->xAxis->setRange(0.0, smem->simTime + 0.001);
    ui->widget_6->yAxis->setRange(yMinWidget6 - 0.001, yMaxWidget6 + 0.001);
    ui->widget_6->replot();
}

void MainWindow::plotWidget7() {
    if (smem->FR_hipJoint < yMinWidget7) { yMinWidget7 = smem->FR_hipJoint; }
    if (smem->FR_hipJoint > yMaxWidget7) { yMaxWidget7 = smem->FR_hipJoint; }
    if (smem->FL_hipJoint < yMinWidget7) { yMinWidget7 = smem->FL_hipJoint; }
    if (smem->FL_hipJoint > yMaxWidget7) { yMaxWidget7 = smem->FL_hipJoint; }
    if (smem->RR_hipJoint < yMinWidget7) { yMinWidget7 = smem->RR_hipJoint; }
    if (smem->RR_hipJoint > yMaxWidget7) { yMaxWidget7 = smem->RR_hipJoint; }
    if (smem->RL_hipJoint < yMinWidget7) { yMinWidget7 = smem->RL_hipJoint; }
    if (smem->RL_hipJoint > yMaxWidget7) { yMaxWidget7 = smem->RL_hipJoint; }
    ui->widget_7->graph(0)->addData(smem->simTime, smem->FR_hipJoint);
    ui->widget_7->graph(1)->addData(smem->simTime, smem->FL_hipJoint);
    ui->widget_7->graph(2)->addData(smem->simTime, smem->RR_hipJoint);
    ui->widget_7->graph(3)->addData(smem->simTime, smem->RL_hipJoint);

    // set axes ranges, so we see all data:
    ui->widget_7->xAxis->setRange(0.0, smem->simTime + 0.001);
    ui->widget_7->yAxis->setRange(yMinWidget7 - 0.001, yMaxWidget7 + 0.001);
    ui->widget_7->replot();
}

void MainWindow::plotWidget8() {
    if (smem->FR_thightJoint < yMinWidget8) { yMinWidget8 = smem->FR_thightJoint; }
    if (smem->FR_thightJoint > yMaxWidget8) { yMaxWidget8 = smem->FR_thightJoint; }
    if (smem->FL_thightJoint < yMinWidget8) { yMinWidget8 = smem->FL_thightJoint; }
    if (smem->FL_thightJoint > yMaxWidget8) { yMaxWidget8 = smem->FL_thightJoint; }
    if (smem->RR_thightJoint < yMinWidget8) { yMinWidget8 = smem->RR_thightJoint; }
    if (smem->RR_thightJoint > yMaxWidget8) { yMaxWidget8 = smem->RR_thightJoint; }
    if (smem->RL_thightJoint < yMinWidget8) { yMinWidget8 = smem->RL_thightJoint; }
    if (smem->RL_thightJoint > yMaxWidget8) { yMaxWidget8 = smem->RL_thightJoint; }
    ui->widget_8->graph(0)->addData(smem->simTime, smem->FR_thightJoint);
    ui->widget_8->graph(1)->addData(smem->simTime, smem->FL_thightJoint);
    ui->widget_8->graph(2)->addData(smem->simTime, smem->RR_thightJoint);
    ui->widget_8->graph(3)->addData(smem->simTime, smem->RL_thightJoint);

    // set axes ranges, so we see all data:
    ui->widget_8->xAxis->setRange(0.0, smem->simTime + 0.001);
    ui->widget_8->yAxis->setRange(yMinWidget8 - 0.001, yMaxWidget8 + 0.001);
    ui->widget_8->replot();
}

void MainWindow::plotWidget9() {
    if (smem->FR_calfJoint < yMinWidget9) { yMinWidget9 = smem->FR_calfJoint; }
    if (smem->FR_calfJoint > yMaxWidget9) { yMaxWidget9 = smem->FR_calfJoint; }
    if (smem->FL_calfJoint < yMinWidget9) { yMinWidget9 = smem->FL_calfJoint; }
    if (smem->FL_calfJoint > yMaxWidget9) { yMaxWidget9 = smem->FL_calfJoint; }
    if (smem->RR_calfJoint < yMinWidget9) { yMinWidget9 = smem->RR_calfJoint; }
    if (smem->RR_calfJoint > yMaxWidget9) { yMaxWidget9 = smem->RR_calfJoint; }
    if (smem->RL_calfJoint < yMinWidget9) { yMinWidget9 = smem->RL_calfJoint; }
    if (smem->RL_calfJoint > yMaxWidget9) { yMaxWidget9 = smem->RL_calfJoint; }
    ui->widget_9->graph(0)->addData(smem->simTime, smem->FR_calfJoint);
    ui->widget_9->graph(1)->addData(smem->simTime, smem->FL_calfJoint);
    ui->widget_9->graph(2)->addData(smem->simTime, smem->RR_calfJoint);
    ui->widget_9->graph(3)->addData(smem->simTime, smem->RL_calfJoint);

    // set axes ranges, so we see all data:
    ui->widget_9->xAxis->setRange(0.0, smem->simTime + 0.001);
    ui->widget_9->yAxis->setRange(yMinWidget9 - 0.001, yMaxWidget9 + 0.001);
    ui->widget_9->replot();
}