/********************************************************************************
** Form generated from reading UI file 'omniwheeldialog.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_OMNIWHEELDIALOG_H
#define UI_OMNIWHEELDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_OmniWheelDialog
{
public:
    QLineEdit *OW_EDIT_T_POS;
    QLabel *OW_LABEL_1;
    QLabel *OW_LABEL_3;
    QLineEdit *OW_EDIT_Y_POS;
    QPushButton *OW_GOTO_DES;
    QLineEdit *OW_EDIT_X_POS;
    QLabel *OW_LABEL_2;
    QPushButton *OW_STOP;
    QPushButton *OW_TRANSFORM_WALK_TO_WHEEL;
    QPushButton *OW_SET_ZERO;
    QLineEdit *OW_EDIT_M_POS_X;
    QLabel *OW_LABEL_5;
    QLabel *OW_LABEL_6;
    QLineEdit *OW_EDIT_Z_POS;
    QFrame *OW_LINE_5;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout;
    QRadioButton *OW_RADIO_POINT;
    QRadioButton *OW_RADIO_APPROACH;
    QPushButton *OW_NORMAL_WALKREADY;
    QLineEdit *OW_EDIT_M_POS_Y;
    QLabel *OW_LABEL_13;
    QPushButton *OW_TRANSFORM_WHEEL_TO_WALK;
    QPushButton *OW_KNEE_GAIN_START;
    QPushButton *OW_KNEE_GAIN_STOP;
    QPushButton *SET_TO_TEST;
    QPushButton *SET_TO_REAL;
    QLineEdit *OW_EDIT_VEL;
    QPushButton *BTN_ROS_MODE;

    void setupUi(QDialog *OmniWheelDialog)
    {
        if (OmniWheelDialog->objectName().isEmpty())
            OmniWheelDialog->setObjectName(QStringLiteral("OmniWheelDialog"));
        OmniWheelDialog->resize(517, 484);
        OW_EDIT_T_POS = new QLineEdit(OmniWheelDialog);
        OW_EDIT_T_POS->setObjectName(QStringLiteral("OW_EDIT_T_POS"));
        OW_EDIT_T_POS->setGeometry(QRect(90, 210, 61, 23));
        OW_LABEL_1 = new QLabel(OmniWheelDialog);
        OW_LABEL_1->setObjectName(QStringLiteral("OW_LABEL_1"));
        OW_LABEL_1->setGeometry(QRect(20, 120, 61, 21));
        OW_LABEL_3 = new QLabel(OmniWheelDialog);
        OW_LABEL_3->setObjectName(QStringLiteral("OW_LABEL_3"));
        OW_LABEL_3->setGeometry(QRect(20, 210, 61, 21));
        OW_EDIT_Y_POS = new QLineEdit(OmniWheelDialog);
        OW_EDIT_Y_POS->setObjectName(QStringLiteral("OW_EDIT_Y_POS"));
        OW_EDIT_Y_POS->setGeometry(QRect(90, 150, 61, 23));
        OW_GOTO_DES = new QPushButton(OmniWheelDialog);
        OW_GOTO_DES->setObjectName(QStringLiteral("OW_GOTO_DES"));
        OW_GOTO_DES->setGeometry(QRect(10, 370, 171, 31));
        OW_EDIT_X_POS = new QLineEdit(OmniWheelDialog);
        OW_EDIT_X_POS->setObjectName(QStringLiteral("OW_EDIT_X_POS"));
        OW_EDIT_X_POS->setGeometry(QRect(90, 120, 61, 23));
        OW_LABEL_2 = new QLabel(OmniWheelDialog);
        OW_LABEL_2->setObjectName(QStringLiteral("OW_LABEL_2"));
        OW_LABEL_2->setGeometry(QRect(20, 150, 61, 21));
        OW_STOP = new QPushButton(OmniWheelDialog);
        OW_STOP->setObjectName(QStringLiteral("OW_STOP"));
        OW_STOP->setGeometry(QRect(10, 410, 171, 31));
        OW_TRANSFORM_WALK_TO_WHEEL = new QPushButton(OmniWheelDialog);
        OW_TRANSFORM_WALK_TO_WHEEL->setObjectName(QStringLiteral("OW_TRANSFORM_WALK_TO_WHEEL"));
        OW_TRANSFORM_WALK_TO_WHEEL->setEnabled(true);
        OW_TRANSFORM_WALK_TO_WHEEL->setGeometry(QRect(10, 40, 81, 51));
        OW_SET_ZERO = new QPushButton(OmniWheelDialog);
        OW_SET_ZERO->setObjectName(QStringLiteral("OW_SET_ZERO"));
        OW_SET_ZERO->setGeometry(QRect(160, 120, 21, 181));
        OW_SET_ZERO->setFlat(false);
        OW_EDIT_M_POS_X = new QLineEdit(OmniWheelDialog);
        OW_EDIT_M_POS_X->setObjectName(QStringLiteral("OW_EDIT_M_POS_X"));
        OW_EDIT_M_POS_X->setGeometry(QRect(90, 250, 61, 23));
        OW_LABEL_5 = new QLabel(OmniWheelDialog);
        OW_LABEL_5->setObjectName(QStringLiteral("OW_LABEL_5"));
        OW_LABEL_5->setGeometry(QRect(20, 250, 71, 21));
        OW_LABEL_6 = new QLabel(OmniWheelDialog);
        OW_LABEL_6->setObjectName(QStringLiteral("OW_LABEL_6"));
        OW_LABEL_6->setGeometry(QRect(20, 180, 61, 21));
        OW_EDIT_Z_POS = new QLineEdit(OmniWheelDialog);
        OW_EDIT_Z_POS->setObjectName(QStringLiteral("OW_EDIT_Z_POS"));
        OW_EDIT_Z_POS->setGeometry(QRect(90, 180, 61, 23));
        OW_LINE_5 = new QFrame(OmniWheelDialog);
        OW_LINE_5->setObjectName(QStringLiteral("OW_LINE_5"));
        OW_LINE_5->setGeometry(QRect(10, 230, 141, 20));
        OW_LINE_5->setFrameShape(QFrame::HLine);
        OW_LINE_5->setFrameShadow(QFrame::Sunken);
        horizontalLayoutWidget = new QWidget(OmniWheelDialog);
        horizontalLayoutWidget->setObjectName(QStringLiteral("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(10, 310, 192, 31));
        horizontalLayout = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        OW_RADIO_POINT = new QRadioButton(horizontalLayoutWidget);
        OW_RADIO_POINT->setObjectName(QStringLiteral("OW_RADIO_POINT"));

        horizontalLayout->addWidget(OW_RADIO_POINT);

        OW_RADIO_APPROACH = new QRadioButton(horizontalLayoutWidget);
        OW_RADIO_APPROACH->setObjectName(QStringLiteral("OW_RADIO_APPROACH"));
        OW_RADIO_APPROACH->setChecked(true);

        horizontalLayout->addWidget(OW_RADIO_APPROACH);

        OW_NORMAL_WALKREADY = new QPushButton(OmniWheelDialog);
        OW_NORMAL_WALKREADY->setObjectName(QStringLiteral("OW_NORMAL_WALKREADY"));
        OW_NORMAL_WALKREADY->setGeometry(QRect(10, 0, 171, 31));
        OW_EDIT_M_POS_Y = new QLineEdit(OmniWheelDialog);
        OW_EDIT_M_POS_Y->setObjectName(QStringLiteral("OW_EDIT_M_POS_Y"));
        OW_EDIT_M_POS_Y->setGeometry(QRect(90, 280, 61, 23));
        OW_LABEL_13 = new QLabel(OmniWheelDialog);
        OW_LABEL_13->setObjectName(QStringLiteral("OW_LABEL_13"));
        OW_LABEL_13->setGeometry(QRect(20, 280, 71, 21));
        OW_TRANSFORM_WHEEL_TO_WALK = new QPushButton(OmniWheelDialog);
        OW_TRANSFORM_WHEEL_TO_WALK->setObjectName(QStringLiteral("OW_TRANSFORM_WHEEL_TO_WALK"));
        OW_TRANSFORM_WHEEL_TO_WALK->setEnabled(true);
        OW_TRANSFORM_WHEEL_TO_WALK->setGeometry(QRect(100, 40, 81, 51));
        OW_KNEE_GAIN_START = new QPushButton(OmniWheelDialog);
        OW_KNEE_GAIN_START->setObjectName(QStringLiteral("OW_KNEE_GAIN_START"));
        OW_KNEE_GAIN_START->setEnabled(false);
        OW_KNEE_GAIN_START->setGeometry(QRect(200, 0, 80, 23));
        OW_KNEE_GAIN_STOP = new QPushButton(OmniWheelDialog);
        OW_KNEE_GAIN_STOP->setObjectName(QStringLiteral("OW_KNEE_GAIN_STOP"));
        OW_KNEE_GAIN_STOP->setEnabled(false);
        OW_KNEE_GAIN_STOP->setGeometry(QRect(290, 0, 80, 23));
        SET_TO_TEST = new QPushButton(OmniWheelDialog);
        SET_TO_TEST->setObjectName(QStringLiteral("SET_TO_TEST"));
        SET_TO_TEST->setEnabled(true);
        SET_TO_TEST->setGeometry(QRect(290, 50, 81, 31));
        SET_TO_REAL = new QPushButton(OmniWheelDialog);
        SET_TO_REAL->setObjectName(QStringLiteral("SET_TO_REAL"));
        SET_TO_REAL->setEnabled(true);
        SET_TO_REAL->setGeometry(QRect(200, 50, 81, 31));
        OW_EDIT_VEL = new QLineEdit(OmniWheelDialog);
        OW_EDIT_VEL->setObjectName(QStringLiteral("OW_EDIT_VEL"));
        OW_EDIT_VEL->setGeometry(QRect(200, 210, 61, 23));
        BTN_ROS_MODE = new QPushButton(OmniWheelDialog);
        BTN_ROS_MODE->setObjectName(QStringLiteral("BTN_ROS_MODE"));
        BTN_ROS_MODE->setGeometry(QRect(250, 300, 171, 111));

        retranslateUi(OmniWheelDialog);

        OW_SET_ZERO->setDefault(false);


        QMetaObject::connectSlotsByName(OmniWheelDialog);
    } // setupUi

    void retranslateUi(QDialog *OmniWheelDialog)
    {
        OmniWheelDialog->setWindowTitle(QApplication::translate("OmniWheelDialog", "Dialog", 0));
        OW_EDIT_T_POS->setText(QApplication::translate("OmniWheelDialog", "0", 0));
        OW_LABEL_1->setText(QApplication::translate("OmniWheelDialog", "X-pos(m)", 0));
        OW_LABEL_3->setText(QApplication::translate("OmniWheelDialog", "Angle(')", 0));
        OW_EDIT_Y_POS->setText(QApplication::translate("OmniWheelDialog", "0", 0));
        OW_GOTO_DES->setText(QApplication::translate("OmniWheelDialog", "Goto Point", 0));
        OW_EDIT_X_POS->setText(QApplication::translate("OmniWheelDialog", "0", 0));
        OW_LABEL_2->setText(QApplication::translate("OmniWheelDialog", "Y-pos(m)", 0));
        OW_STOP->setText(QApplication::translate("OmniWheelDialog", "E-Stop", 0));
        OW_TRANSFORM_WALK_TO_WHEEL->setText(QApplication::translate("OmniWheelDialog", "to Wheel", 0));
        OW_SET_ZERO->setText(QApplication::translate("OmniWheelDialog", "#", 0));
        OW_EDIT_M_POS_X->setText(QApplication::translate("OmniWheelDialog", "0.6", 0));
        OW_LABEL_5->setText(QApplication::translate("OmniWheelDialog", "Margin-x", 0));
        OW_LABEL_6->setText(QApplication::translate("OmniWheelDialog", "Z-pos(m)", 0));
        OW_EDIT_Z_POS->setText(QApplication::translate("OmniWheelDialog", "0", 0));
        OW_RADIO_POINT->setText(QApplication::translate("OmniWheelDialog", "Point go", 0));
        OW_RADIO_APPROACH->setText(QApplication::translate("OmniWheelDialog", "Approach", 0));
        OW_NORMAL_WALKREADY->setText(QApplication::translate("OmniWheelDialog", "WalkReady", 0));
        OW_EDIT_M_POS_Y->setText(QApplication::translate("OmniWheelDialog", "0.35", 0));
        OW_LABEL_13->setText(QApplication::translate("OmniWheelDialog", "Margin-y", 0));
        OW_TRANSFORM_WHEEL_TO_WALK->setText(QApplication::translate("OmniWheelDialog", "to Walk", 0));
        OW_KNEE_GAIN_START->setText(QApplication::translate("OmniWheelDialog", "Release", 0));
        OW_KNEE_GAIN_STOP->setText(QApplication::translate("OmniWheelDialog", "return", 0));
        SET_TO_TEST->setText(QApplication::translate("OmniWheelDialog", "set Test", 0));
        SET_TO_REAL->setText(QApplication::translate("OmniWheelDialog", "set Real", 0));
        OW_EDIT_VEL->setText(QApplication::translate("OmniWheelDialog", "360", 0));
        BTN_ROS_MODE->setText(QApplication::translate("OmniWheelDialog", "ROS MODE", 0));
    } // retranslateUi

};

namespace Ui {
    class OmniWheelDialog: public Ui_OmniWheelDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_OMNIWHEELDIALOG_H
