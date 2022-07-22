//
// Created by jaehoon on 22. 4. 26.
//

#ifndef RAISIM_SIMULATIONMAINWINDOW_H
#define RAISIM_MAINWINDOW_H
#include <QMainWindow>
#include <QTimer>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    bool buttonCANInit = false;
    bool buttonRaisimInit = false;
    bool buttonMotorOn = false;
    bool buttonMotorOff = false;
    bool buttonStartControl = false;
    bool buttonStopControl = false;
    bool buttonGenerateCubicTrajectory = false;
    bool buttonGenerateSinTrajectory = false;
    bool buttonJump = false;
    bool buttonZeroing = false;
    bool isOperationEnd = false;

    double yMinWidget1;
    double yMaxWidget1;
    double yMinWidget2;
    double yMaxWidget2;
    double yMinWidget3;
    double yMaxWidget3;
    double yMinWidget4;
    double yMaxWidget4;
    double yMinWidget5;
    double yMaxWidget5;
    double yMinWidget6;
    double yMaxWidget6;
    double intervalTime = 20.0;

    QTimer dataTimer;

public slots:
    void plotWidget1();
    void plotWidget2();
    void plotWidget3();
    void plotWidget4();
    void plotWidget5();
    void plotWidget6();
    void realtimeDataSlot();

private slots:
    void on_pushButton_1_clicked();
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();
    void on_pushButton_4_clicked();
    void on_pushButton_5_clicked();
    void on_pushButton_6_clicked();
    void on_pushButton_7_clicked();
    void on_pushButton_8_clicked();
    void on_pushButton_9_clicked();
    void on_pushButton_10_clicked();

private:
    Ui::MainWindow *ui;
};

#endif //RAISIM_SIMULATIONMAINWINDOW_H
