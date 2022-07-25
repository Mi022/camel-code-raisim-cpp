//
// Created by jaehoon on 22. 4. 26.
//

#ifndef RAISIM_UI_SIMULATIONMAINWINDOW_H
#define RAISIM_UI_MAINWINDOW_H
/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>
#include "include/SimulationUI/qcustomplot.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QCustomPlot *widget_1;
    QCustomPlot *widget_2;
    QCustomPlot *widget_3;
    QCustomPlot *widget_4;
    QCustomPlot *widget_5;
    QCustomPlot *widget_6;
    QPushButton *pushButton_1;
    QPushButton *pushButton_2;
    QPushButton *pushButton_3;
    QPushButton *pushButton_4;
    QPushButton *pushButton_5;
    QPushButton *pushButton_6;
    QPushButton *pushButton_7;
    QPushButton *pushButton_8;
    QPushButton *pushButton_9;
    QPushButton *pushButton_10;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1808, 1271);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        widget_1 = new QCustomPlot(centralwidget);
        widget_1->setObjectName(QString::fromUtf8("widget_1"));
        widget_1->setGeometry(QRect(20, 10, 751, 391));
        widget_2 = new QCustomPlot(centralwidget);
        widget_2->setObjectName(QString::fromUtf8("widget_2"));
        widget_2->setGeometry(QRect(780, 10, 751, 391));
        widget_3 = new QCustomPlot(centralwidget);
        widget_3->setObjectName(QString::fromUtf8("widget_3"));
        widget_3->setGeometry(QRect(20, 410, 751, 391));
        widget_4 = new QCustomPlot(centralwidget);
        widget_4->setObjectName(QString::fromUtf8("widget_4"));
        widget_4->setGeometry(QRect(780, 410, 751, 391));
        widget_5 = new QCustomPlot(centralwidget);
        widget_5->setObjectName(QString::fromUtf8("widget_5"));
        widget_5->setGeometry(QRect(20, 810, 751, 401));
        widget_6 = new QCustomPlot(centralwidget);
        widget_6->setObjectName(QString::fromUtf8("widget_6"));
        widget_6->setGeometry(QRect(780, 810, 751, 401));
        pushButton_1 = new QPushButton(centralwidget);
        pushButton_1->setObjectName(QString::fromUtf8("pushButton_1"));
        pushButton_1->setGeometry(QRect(1560, 10, 111, 61));
        pushButton_2 = new QPushButton(centralwidget);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        pushButton_2->setGeometry(QRect(1690, 10, 111, 61));
        pushButton_3 = new QPushButton(centralwidget);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));
        pushButton_3->setGeometry(QRect(1560, 90, 111, 61));
        pushButton_4 = new QPushButton(centralwidget);
        pushButton_4->setObjectName(QString::fromUtf8("pushButton_4"));
        pushButton_4->setGeometry(QRect(1690, 90, 111, 61));
        pushButton_5 = new QPushButton(centralwidget);
        pushButton_5->setObjectName(QString::fromUtf8("pushButton_5"));
        pushButton_5->setGeometry(QRect(1560, 170, 111, 61));
        pushButton_6 = new QPushButton(centralwidget);
        pushButton_6->setObjectName(QString::fromUtf8("pushButton_6"));
        pushButton_6->setGeometry(QRect(1690, 170, 111, 61));
        pushButton_7 = new QPushButton(centralwidget);
        pushButton_7->setObjectName(QString::fromUtf8("pushButton_7"));
        pushButton_7->setGeometry(QRect(1560, 250, 241, 61));
        pushButton_8 = new QPushButton(centralwidget);
        pushButton_8->setObjectName(QString::fromUtf8("pushButton_8"));
        pushButton_8->setGeometry(QRect(1560, 330, 241, 61));
        pushButton_9 = new QPushButton(centralwidget);
        pushButton_9->setObjectName(QString::fromUtf8("pushButton_9"));
        pushButton_9->setGeometry(QRect(1560, 410, 241, 61));
        pushButton_10 = new QPushButton(centralwidget);
        pushButton_10->setObjectName(QString::fromUtf8("pushButton_10"));
        pushButton_10->setGeometry(QRect(1560, 490, 241, 61));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1808, 22));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
        pushButton_1->setText(QApplication::translate("MainWindow", "CAN Init", nullptr));
        pushButton_2->setText(QApplication::translate("MainWindow", "Raisim Init", nullptr));
        pushButton_3->setText(QApplication::translate("MainWindow", "Motor On", nullptr));
        pushButton_4->setText(QApplication::translate("MainWindow", "Motor Off", nullptr));
        pushButton_5->setText(QApplication::translate("MainWindow", "Start Control", nullptr));
        pushButton_6->setText(QApplication::translate("MainWindow", "Stop Control", nullptr));
        pushButton_7->setText(QApplication::translate("MainWindow", "Generate new cubic trajectory", nullptr));
        pushButton_8->setText(QApplication::translate("MainWindow", "Generate new sin trajectory", nullptr));
        pushButton_9->setText(QApplication::translate("MainWindow", "Jump", nullptr));
        pushButton_10->setText(QApplication::translate("MainWindow", "Zeroing", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif //RAISIM_UI_SIMULATIONMAINWINDOW_H
