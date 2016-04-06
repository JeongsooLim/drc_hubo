/********************************************************************************
** Form generated from reading UI file 'GUIMainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GUIMAINWINDOW_H
#define UI_GUIMAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_GUIMainWindow
{
public:
    QAction *actionLAN;
    QAction *actionPODOAL;
    QAction *actionJOINT;
    QAction *actionSENSOR;
    QAction *actionDUMMY;
    QAction *actionMODEL;
    QWidget *centralWidget;
    QTabWidget *MAIN_TAB;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *GUIMainWindow)
    {
        if (GUIMainWindow->objectName().isEmpty())
            GUIMainWindow->setObjectName(QStringLiteral("GUIMainWindow"));
        GUIMainWindow->resize(800, 680);
        actionLAN = new QAction(GUIMainWindow);
        actionLAN->setObjectName(QStringLiteral("actionLAN"));
        actionLAN->setCheckable(true);
        QIcon icon;
        icon.addFile(QStringLiteral("../SHARE/GUI/icon/LAN_OFF_BLACK.png"), QSize(), QIcon::Normal, QIcon::Off);
        icon.addFile(QStringLiteral("../SHARE/GUI/icon/LAN_OFF_BLUE.png"), QSize(), QIcon::Normal, QIcon::On);
        actionLAN->setIcon(icon);
        actionPODOAL = new QAction(GUIMainWindow);
        actionPODOAL->setObjectName(QStringLiteral("actionPODOAL"));
        actionPODOAL->setCheckable(true);
        QIcon icon1;
        icon1.addFile(QStringLiteral("../SHARE/GUI/icon/MODULE_BLACK.png"), QSize(), QIcon::Normal, QIcon::Off);
        icon1.addFile(QStringLiteral("../SHARE/GUI/icon/MODULE_BLUE.png"), QSize(), QIcon::Normal, QIcon::On);
        actionPODOAL->setIcon(icon1);
        actionJOINT = new QAction(GUIMainWindow);
        actionJOINT->setObjectName(QStringLiteral("actionJOINT"));
        actionJOINT->setCheckable(true);
        QIcon icon2;
        icon2.addFile(QStringLiteral("../SHARE/GUI/icon/JOINT_BLACK.png"), QSize(), QIcon::Normal, QIcon::Off);
        icon2.addFile(QStringLiteral("../SHARE/GUI/icon/JOINT_BLUE.png"), QSize(), QIcon::Normal, QIcon::On);
        actionJOINT->setIcon(icon2);
        actionSENSOR = new QAction(GUIMainWindow);
        actionSENSOR->setObjectName(QStringLiteral("actionSENSOR"));
        actionSENSOR->setCheckable(true);
        QIcon icon3;
        icon3.addFile(QStringLiteral("../SHARE/GUI/icon/SENSOR_BLACK.png"), QSize(), QIcon::Normal, QIcon::Off);
        icon3.addFile(QStringLiteral("../SHARE/GUI/icon/SENSOR_BLUE.png"), QSize(), QIcon::Normal, QIcon::On);
        actionSENSOR->setIcon(icon3);
        actionDUMMY = new QAction(GUIMainWindow);
        actionDUMMY->setObjectName(QStringLiteral("actionDUMMY"));
        actionDUMMY->setEnabled(false);
        actionMODEL = new QAction(GUIMainWindow);
        actionMODEL->setObjectName(QStringLiteral("actionMODEL"));
        actionMODEL->setCheckable(true);
        QIcon icon4;
        icon4.addFile(QStringLiteral("../SHARE/GUI/icon/SIMULATOR_BLACK.png"), QSize(), QIcon::Normal, QIcon::Off);
        icon4.addFile(QStringLiteral("../SHARE/GUI/icon/SIMULATOR_BLUE.png"), QSize(), QIcon::Normal, QIcon::On);
        actionMODEL->setIcon(icon4);
        centralWidget = new QWidget(GUIMainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        MAIN_TAB = new QTabWidget(centralWidget);
        MAIN_TAB->setObjectName(QStringLiteral("MAIN_TAB"));
        MAIN_TAB->setGeometry(QRect(10, 10, 681, 611));
        GUIMainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(GUIMainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 800, 25));
        GUIMainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(GUIMainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(mainToolBar->sizePolicy().hasHeightForWidth());
        mainToolBar->setSizePolicy(sizePolicy);
        mainToolBar->setStyleSheet(QStringLiteral("background-color: rgb(202, 203, 210);"));
        mainToolBar->setMovable(false);
        mainToolBar->setIconSize(QSize(80, 80));
        GUIMainWindow->addToolBar(Qt::LeftToolBarArea, mainToolBar);
        statusBar = new QStatusBar(GUIMainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        GUIMainWindow->setStatusBar(statusBar);

        mainToolBar->addAction(actionLAN);
        mainToolBar->addAction(actionPODOAL);
        mainToolBar->addAction(actionDUMMY);
        mainToolBar->addAction(actionJOINT);
        mainToolBar->addAction(actionSENSOR);
        mainToolBar->addAction(actionMODEL);

        retranslateUi(GUIMainWindow);

        MAIN_TAB->setCurrentIndex(-1);


        QMetaObject::connectSlotsByName(GUIMainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *GUIMainWindow)
    {
        GUIMainWindow->setWindowTitle(QApplication::translate("GUIMainWindow", "PODO GUI -- Rainbow Robotics", 0));
        actionLAN->setText(QApplication::translate("GUIMainWindow", "LAN Connection", 0));
#ifndef QT_NO_TOOLTIP
        actionLAN->setToolTip(QApplication::translate("GUIMainWindow", "LAN Connection", 0));
#endif // QT_NO_TOOLTIP
        actionPODOAL->setText(QApplication::translate("GUIMainWindow", "PODOAL Control", 0));
#ifndef QT_NO_TOOLTIP
        actionPODOAL->setToolTip(QApplication::translate("GUIMainWindow", "PODOAL Control", 0));
#endif // QT_NO_TOOLTIP
        actionJOINT->setText(QApplication::translate("GUIMainWindow", "JOINT Pannel", 0));
#ifndef QT_NO_TOOLTIP
        actionJOINT->setToolTip(QApplication::translate("GUIMainWindow", "JOINT Pannel", 0));
#endif // QT_NO_TOOLTIP
        actionSENSOR->setText(QApplication::translate("GUIMainWindow", "SENSOR Pannel", 0));
#ifndef QT_NO_TOOLTIP
        actionSENSOR->setToolTip(QApplication::translate("GUIMainWindow", "SENSOR Pannel", 0));
#endif // QT_NO_TOOLTIP
        actionDUMMY->setText(QString());
        actionMODEL->setText(QApplication::translate("GUIMainWindow", "MODEL Pannel", 0));
#ifndef QT_NO_TOOLTIP
        actionMODEL->setToolTip(QApplication::translate("GUIMainWindow", "MODEL Pannel", 0));
#endif // QT_NO_TOOLTIP
    } // retranslateUi

};

namespace Ui {
    class GUIMainWindow: public Ui_GUIMainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GUIMAINWINDOW_H
