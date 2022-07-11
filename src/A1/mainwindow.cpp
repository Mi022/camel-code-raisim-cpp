#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>

MainWindow *MainUI;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    MainUI = this;
    ui->setupUi(this);

    for (int i = 0; i < 201; ++i) {
        data_x[i] = 0;
        data_y1[i] = 0;
        data_y2[i] = 0;
    }

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


void MainWindow::plotWidget1() {
    QVector<double> x(201);
    QVector<double> y1(201);
    QVector<double> y1_desired(201);
    for (int i = 0; i < 201; ++i) {
        x[i] = data_x[i];
        y1[i] = data_y1[i];
        y1_desired[i] = data_y1_desired[i];
        if (y1[i] < data_widget1_min) { data_widget1_min = y1[i]; }
        if (y1[i] > data_widget1_max) { data_widget1_max = y1[i]; }
        if (y1_desired[i] < data_widget1_min) { data_widget1_min = y1_desired[i]; }
        if (y1_desired[i] > data_widget1_max) { data_widget1_max = y1_desired[i]; }
    }
    ui->widget_1->graph(0)->addData(x, y1);
    ui->widget_1->graph(1)->addData(x, y1_desired);
    // give the axes some labels:
    ui->widget_1->xAxis->setLabel("time [sec]");
    ui->widget_1->yAxis->setLabel("position");
    // set axes ranges, so we see all data:
    ui->widget_1->xAxis->setRange(0.0, x[MainUI->data_idx - 1] + 0.001);
    ui->widget_1->yAxis->setRange(data_widget1_min - 0.001, data_widget1_max + 0.001);
    ui->widget_1->replot();
}

void MainWindow::plotWidget2() {
    QVector<double> x(201);
    QVector<double> y2(201);
    QVector<double> y2_desired(201);
    for (int i = 0; i < 201; ++i) {
        x[i] = data_x[i];
        y2[i] = data_y2[i];
        y2_desired[i] = data_y2_desired[i];
        if (y2[i] < data_widget2_min) { data_widget2_min = y2[i]; }
        if (y2[i] > data_widget2_max) { data_widget2_max = y2[i]; }
        if (y2_desired[i] < data_widget2_min) { data_widget2_min = y2_desired[i]; }
        if (y2_desired[i] > data_widget2_max) { data_widget2_max = y2_desired[i]; }
    }
    ui->widget_2->graph(0)->addData(x, y2);
    ui->widget_2->graph(1)->addData(x, y2_desired);
    // give the axes some labels:
    ui->widget_2->xAxis->setLabel("time [sec]");
    ui->widget_2->yAxis->setLabel("position");
    // set axes ranges, so we see all data:
    ui->widget_2->xAxis->setRange(0.0, x[MainUI->data_idx - 1] + 0.001);
    ui->widget_2->yAxis->setRange(data_widget2_min - 0.001, data_widget2_max + 0.001);
    ui->widget_2->replot();
}

void MainWindow::plotWidget3() {
    QVector<double> x(201);
    QVector<double> y3_red(201);
    QVector<double> y3_blue(201);
    for (int i = 0; i < 201; ++i) {
        x[i] = data_x[i];
        y3_red[i] = data_y3[i];
        y3_blue[i] = data_y3_desired[i];
        if (y3_red[i] < data_widget3_min) { data_widget3_min = y3_red[i]; }
        if (y3_red[i] > data_widget3_max) { data_widget3_max = y3_red[i]; }
        if (y3_blue[i] < data_widget3_min) { data_widget3_min = y3_blue[i]; }
        if (y3_blue[i] > data_widget3_max) { data_widget3_max = y3_blue[i]; }

    }
    ui->widget_3->graph(0)->addData(x, y3_blue);
    ui->widget_3->graph(1)->addData(x, y3_red);
    // give the axes some labels:
    ui->widget_3->xAxis->setLabel("time [sec]");
    ui->widget_3->yAxis->setLabel("position");
    // set axes ranges, so we see all data:
    ui->widget_3->xAxis->setRange(0.0, x[MainUI->data_idx - 1] + 0.001);
    ui->widget_3->yAxis->setRange(data_widget3_min - 0.1, data_widget3_max + 0.1);
    ui->widget_3->replot();
}

void MainWindow::plotWidget4() {
    QVector<double> x(201);
    QVector<double> y4(201);
    QVector<double> y4_desired(201);
    for (int i = 0; i < 201; ++i) {
        x[i] = data_x[i];
        y4[i] = data_y4[i];
        y4_desired[i] = data_y4_desired[i];
        if (y4[i] < data_widget4_min) { data_widget4_min = y4[i]; }
        if (y4[i] > data_widget4_max) { data_widget4_max = y4[i]; }
        if (y4_desired[i] < data_widget4_min) { data_widget4_min = y4_desired[i]; }
        if (y4_desired[i] > data_widget4_max) { data_widget4_max = y4_desired[i]; }
    }
    ui->widget_4->graph(0)->addData(x, y4);
    ui->widget_4->graph(1)->addData(x, y4_desired);
    // give the axes some labels:
    ui->widget_4->xAxis->setLabel("time [sec]");
    ui->widget_4->yAxis->setLabel("rotation");
    // set axes ranges, so we see all data:
    ui->widget_4->xAxis->setRange(0.0, x[MainUI->data_idx - 1] + 0.001);
    ui->widget_4->yAxis->setRange(data_widget4_min - 0.001, data_widget4_max + 0.001);
    ui->widget_4->replot();
}

void MainWindow::plotWidget5() {
    QVector<double> x(201);
    QVector<double> y5(201);
    QVector<double> y5_desired(201);
    for (int i = 0; i < 201; ++i) {
        x[i] = data_x[i];
        y5[i] = data_y5[i];
        y5_desired[i] = data_y5_desired[i];
        if (y5[i] < data_widget5_min) { data_widget5_min = y5[i]; }
        if (y5[i] > data_widget5_max) { data_widget5_max = y5[i]; }
        if (y5_desired[i] < data_widget5_min) { data_widget5_min = y5_desired[i]; }
        if (y5_desired[i] > data_widget5_max) { data_widget5_max = y5_desired[i]; }
    }
    ui->widget_5->graph(0)->addData(x, y5);
    ui->widget_5->graph(1)->addData(x, y5_desired);
    // give the axes some labels:
    ui->widget_5->xAxis->setLabel("time [sec]");
    ui->widget_5->yAxis->setLabel("rotation");
    // set axes ranges, so we see all data:
    ui->widget_5->xAxis->setRange(0.0, x[MainUI->data_idx - 1] + 0.001);
    ui->widget_5->yAxis->setRange(data_widget5_min - 0.001, data_widget5_max + 0.001);
    ui->widget_5->replot();
}

void MainWindow::plotWidget6() {
    QVector<double> x(201);
    QVector<double> y6(201);
    QVector<double> y6_desired(201);
    for (int i = 0; i < 201; ++i) {
        x[i] = data_x[i];
        y6[i] = data_y6[i];
        y6_desired[i] = data_y6_desired[i];
        if (y6[i] < data_widget6_min) { data_widget6_min = y6[i]; }
        if (y6[i] > data_widget6_max) { data_widget6_max = y6[i]; }
        if (y6_desired[i] < data_widget6_min) { data_widget6_min = y6_desired[i]; }
        if (y6_desired[i] > data_widget6_max) { data_widget6_max = y6_desired[i]; }

    }
    ui->widget_6->graph(0)->addData(x, y6);
    ui->widget_6->graph(1)->addData(x, y6_desired);
    // give the axes some labels:
    ui->widget_6->xAxis->setLabel("time [sec]");
    ui->widget_6->yAxis->setLabel("rotation");
    // set axes ranges, so we see all data:
    ui->widget_6->xAxis->setRange(0.0, x[MainUI->data_idx - 1] + 0.001);
    ui->widget_6->yAxis->setRange(data_widget6_min - 0.1, data_widget6_max + 0.1);
    ui->widget_6->replot();
}

void MainWindow::plotWidget7() {
    QVector<double> x(201);
    QVector<double> fr(201);
    QVector<double> fl(201);
    QVector<double> rr(201);
    QVector<double> rl(201);
    for (int i = 0; i < 201; ++i) {
        x[i] = data_x[i];
        fr[i] = data_hip_fr[i];
        fl[i] = data_hip_fl[i];
        rr[i] = data_hip_rr[i];
        rl[i] = data_hip_rl[i];
        if (fr[i] < data_hip_min) { data_hip_min = fr[i]; }
        if (fr[i] > data_hip_max) { data_hip_max = fr[i]; }
        if (fl[i] < data_hip_min) { data_hip_min = fl[i]; }
        if (fl[i] > data_hip_max) { data_hip_max = fl[i]; }
        if (rr[i] < data_hip_min) { data_hip_min = rr[i]; }
        if (rr[i] > data_hip_max) { data_hip_max = rr[i]; }
        if (rl[i] < data_hip_min) { data_hip_min = rl[i]; }
        if (rl[i] > data_hip_max) { data_hip_max = rl[i]; }
    }
    ui->widget_7->graph(0)->addData(x, fr);
    ui->widget_7->graph(1)->addData(x, fl);
    ui->widget_7->graph(2)->addData(x, rr);
    ui->widget_7->graph(3)->addData(x, rl);
    // give the axes some labels:
    ui->widget_7->xAxis->setLabel("time [sec]");
    ui->widget_7->yAxis->setLabel("torque");
    // set axes ranges, so we see all data:
    ui->widget_7->xAxis->setRange(0.0, x[MainUI->data_idx - 1] + 0.001);
    ui->widget_7->yAxis->setRange(data_hip_min - 0.001, data_hip_max + 0.001);
    ui->widget_7->replot();
}

void MainWindow::plotWidget8() {
    QVector<double> x(201);
    QVector<double> fr(201);
    QVector<double> fl(201);
    QVector<double> rr(201);
    QVector<double> rl(201);
    for (int i = 0; i < 201; ++i) {
        x[i] = data_x[i];
        fr[i] = data_thigh_fr[i];
        fl[i] = data_thigh_fl[i];
        rr[i] = data_thigh_rr[i];
        rl[i] = data_thigh_rl[i];
        if (fr[i] < data_thigh_min) { data_thigh_min = fr[i]; }
        if (fr[i] > data_thigh_max) { data_thigh_max = fr[i]; }
        if (fl[i] < data_thigh_min) { data_thigh_min = fl[i]; }
        if (fl[i] > data_thigh_max) { data_thigh_max = fl[i]; }
        if (rr[i] < data_thigh_min) { data_thigh_min = rr[i]; }
        if (rr[i] > data_thigh_max) { data_thigh_max = rr[i]; }
        if (rl[i] < data_thigh_min) { data_thigh_min = rl[i]; }
        if (rl[i] > data_thigh_max) { data_thigh_max = rl[i]; }
    }
    ui->widget_8->graph(0)->addData(x, fr);
    ui->widget_8->graph(1)->addData(x, fl);
    ui->widget_8->graph(2)->addData(x, rr);
    ui->widget_8->graph(3)->addData(x, rl);
    // give the axes some labels:
    ui->widget_8->xAxis->setLabel("time [sec]");
    ui->widget_8->yAxis->setLabel("torque");
    // set axes ranges, so we see all data:
    ui->widget_8->xAxis->setRange(0.0, x[MainUI->data_idx - 1] + 0.001);
    ui->widget_8->yAxis->setRange(data_thigh_min - 0.001, data_thigh_max + 0.001);
    ui->widget_8->replot();
}

void MainWindow::plotWidget9() {
    QVector<double> x(201);
    QVector<double> fr(201);
    QVector<double> fl(201);
    QVector<double> rr(201);
    QVector<double> rl(201);
    for (int i = 0; i < 201; ++i) {
        x[i] = data_x[i];
        fr[i] = data_calf_fr[i];
        fl[i] = data_calf_fl[i];
        rr[i] = data_calf_rr[i];
        rl[i] = data_calf_rl[i];
        if (fr[i] < data_calf_min) { data_calf_min = fr[i]; }
        if (fr[i] > data_calf_max) { data_calf_max = fr[i]; }
        if (fl[i] < data_calf_min) { data_calf_min = fl[i]; }
        if (fl[i] > data_calf_max) { data_calf_max = fl[i]; }
        if (rr[i] < data_calf_min) { data_calf_min = rr[i]; }
        if (rr[i] > data_calf_max) { data_calf_max = rr[i]; }
        if (rl[i] < data_calf_min) { data_calf_min = rl[i]; }
        if (rl[i] > data_calf_max) { data_calf_max = rl[i]; }
    }
    ui->widget_9->graph(0)->addData(x, fr);
    ui->widget_9->graph(1)->addData(x, fl);
    ui->widget_9->graph(2)->addData(x, rr);
    ui->widget_9->graph(3)->addData(x, rl);
    // give the axes some labels:
    ui->widget_9->xAxis->setLabel("time [sec]");
    ui->widget_9->yAxis->setLabel("torque");
    // set axes ranges, so we see all data:
    ui->widget_9->xAxis->setRange(0.0, x[MainUI->data_idx - 1] + 0.001);
    ui->widget_9->yAxis->setRange(data_calf_min - 0.001, data_calf_max + 0.001);
    ui->widget_9->replot();
}