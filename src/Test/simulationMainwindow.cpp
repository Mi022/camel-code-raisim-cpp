//
// Created by jaehoon on 22. 4. 26.
//

#include "simulationMainwindow.h"
#include "ui_simulationMainwindow.h"
#include <iostream>

MainWindow *MainUI;

MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::MainWindow) {
    MainUI = this;
    ui->setupUi(this);
    for (int i = 0; i < 201; ++i) {
        data_x[i] = 0;
        data_y1[i] = 0;
        data_y2[i] = 0;
    }
    ui->widget->legend->setVisible(true);
    ui->widget->legend->setFont(QFont("Helvetica", 9));
    ui->widget->addGraph();
    ui->widget->graph(0)->setName("position");
    ui->widget->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget->addGraph();
    ui->widget->graph(1)->setName("desired position");
    ui->widget->graph(1)->setPen(QPen(QColor(255, 0, 0)));

    ui->widget_2->legend->setVisible(true);
    ui->widget_2->legend->setFont(QFont("Helvetica", 9));
    ui->widget_2->addGraph();
    ui->widget_2->graph(0)->setName("velocity");
    ui->widget_2->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_2->addGraph();
    ui->widget_2->graph(1)->setName("desired velocity");
    ui->widget_2->graph(1)->setPen(QPen(QColor(255, 0, 0)));

    ui->widget_3->legend->setVisible(true);
    ui->widget_3->legend->setFont(QFont("Helvetica", 9));
    ui->widget_3->addGraph();
    ui->widget_3->graph(0)->setName("torque hip");
    ui->widget_3->graph(0)->setPen(QPen(QColor(0, 0, 255)));
    ui->widget_3->addGraph();
    ui->widget_3->graph(1)->setName("torque knee");
    ui->widget_3->graph(1)->setPen(QPen(QColor(255, 0, 0)));

}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::on_pushButton_clicked() {
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
    ui->widget->graph(0)->addData(x, y1);
    ui->widget->graph(1)->addData(x, y1_desired);
    // give the axes some labels:
    ui->widget->xAxis->setLabel("time [sec]");
    ui->widget->yAxis->setLabel("position");
    // set axes ranges, so we see all data:
    ui->widget->xAxis->setRange(0.0, x[MainUI->data_idx - 1] + 0.001);
    ui->widget->yAxis->setRange(data_widget1_min - 0.001, data_widget1_max + 0.001);
    ui->widget->replot();
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
    ui->widget_2->yAxis->setLabel("velocity");
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
        y3_red[i] = data_y3_red[i];
        y3_blue[i] = data_y3_blue[i];
        if (y3_red[i] < data_widget3_min) { data_widget3_min = y3_red[i]; }
        if (y3_red[i] > data_widget3_max) { data_widget3_max = y3_red[i]; }
        if (y3_blue[i] < data_widget3_min) { data_widget3_min = y3_blue[i]; }
        if (y3_blue[i] > data_widget3_max) { data_widget3_max = y3_blue[i]; }

    }
    ui->widget_3->graph(0)->addData(x, y3_blue);
    ui->widget_3->graph(1)->addData(x, y3_red);
    // give the axes some labels:
    ui->widget_3->xAxis->setLabel("time [sec]");
    ui->widget_3->yAxis->setLabel("torque");
    // set axes ranges, so we see all data:
    ui->widget_3->xAxis->setRange(0.0, x[MainUI->data_idx - 1] + 0.001);
    ui->widget_3->yAxis->setRange(data_widget3_min - 0.1, data_widget3_max + 0.1);
    ui->widget_3->replot();
}

