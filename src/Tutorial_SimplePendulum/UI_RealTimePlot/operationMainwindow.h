//
// Created by jaehoon on 22. 7. 19.
//

#ifndef RAISIM_OPERATIONMAINWINDOW_H
#define RAISIM_OPERATIONMAINWINDOW_H

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
    double yMinWidget1;
    double yMaxWidget1;
    double yMinWidget2;
    double yMaxWidget2;
    double yMinWidget3;
    double yMaxWidget3;
    QTimer dataTimer;

public slots:
    void plotWidget1();
    void plotWidget2();
    void plotWidget3();
    void realtimeDataSlot();

private slots:
    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;
};


#endif //RAISIM_OPERATIONMAINWINDOW_H
