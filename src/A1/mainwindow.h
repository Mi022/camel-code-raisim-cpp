#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
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
    double data_y3[201];
    double data_y3_desired[201];
    double data_widget3_min = -0.01;
    double data_widget3_max = 0.01;

    double data_y4[201];
    double data_y4_desired[201];
    double data_widget4_min = -0.01;
    double data_widget4_max = 0.01;
    double data_y5[201];
    double data_y5_desired[201];
    double data_widget5_min = -0.01;
    double data_widget5_max = 0.01;
    double data_y6[201];
    double data_y6_desired[201];
    double data_widget6_min = -0.01;
    double data_widget6_max = 0.01;

    double data_hip_fr[201];
    double data_hip_fl[201];
    double data_hip_rr[201];
    double data_hip_rl[201];
    double data_hip_min = -0.01;
    double data_hip_max = 0.01;

    double data_thigh_fr[201];
    double data_thigh_fl[201];
    double data_thigh_rr[201];
    double data_thigh_rl[201];
    double data_thigh_min = -0.01;
    double data_thigh_max = 0.01;

    double data_calf_fr[201];
    double data_calf_fl[201];
    double data_calf_rr[201];
    double data_calf_rl[201];
    double data_calf_min = -0.01;
    double data_calf_max = 0.01;

public slots:
    void plotWidget1();
    void plotWidget2();
    void plotWidget3();

    void plotWidget4();
    void plotWidget5();
    void plotWidget6();

    void plotWidget7();
    void plotWidget8();
    void plotWidget9();


private slots:
    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
