//
// Created by ljm on 22. 7. 7.
//

#ifndef RAISIM_WHEELEDROBOTSIMULATIONWINDOW_H
#define RAISIM_WHEELEDROBOTSIMULATIONWINDOW_H

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
    bool button2 = false;
    bool button3 = false;
    bool button4 = false;
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
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();
    void on_pushButton_4_clicked();

private:
    Ui::MainWindow *ui;
};

#endif //RAISIM_WHEELEDROBOTSIMULATIONWINDOW_H
