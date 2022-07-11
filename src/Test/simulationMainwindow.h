//
// Created by jaehoon on 22. 4. 26.
//

#ifndef RAISIM_SIMULATIONMAINWINDOW_H
#define RAISIM_SIMULATIONMAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    bool button1 = false;
    int data_idx = 0;
    double data_x[201];
    double data_y1[201];
    double data_y1_desired[201];
    double data_widget1_min = -0.01;
    double data_widget1_max = 0.01;
    double data_y2[201];
    double data_y2_desired[201];
    double data_widget2_min = -0.01;
    double data_widget2_max = 0.01;
    double data_y3_red[201];
    double data_y3_blue[201];
    double data_widget3_min = -0.01;
    double data_widget3_max = 0.01;

public slots:
    void plotWidget1();
    void plotWidget2();
    void plotWidget3();

private slots:
    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;
};

#endif //RAISIM_SIMULATIONMAINWINDOW_H
