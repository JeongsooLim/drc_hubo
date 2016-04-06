/********************************************************************************
** Form generated from reading UI file 'TutorialDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TUTORIALDIALOG_H
#define UI_TUTORIALDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_TutorialDialog
{
public:
    QGroupBox *groupBox_3;
    QLineEdit *LE_SCAN_START;
    QLineEdit *LE_SCAN_END;
    QLineEdit *LE_SCAN_TIME;
    QPushButton *BTN_SCAN;
    QLineEdit *LE_GOTO_ANGLE;
    QPushButton *BTN_GOTO;
    QLabel *label_7;
    QLabel *label_8;
    QLabel *label_9;
    QLabel *label_10;
    QGroupBox *groupBox_2;
    QPushButton *BTN_WALK_READY;
    QFrame *line;
    QPushButton *BTN_TO_WHEEL;
    QPushButton *BTN_TO_WALK;
    QPushButton *BTN_HOME_POS;
    QGroupBox *groupBox_4;
    QPushButton *BTN_TEST;
    QGroupBox *groupBox_5;
    QPushButton *BTN_GUIMODE;
    QPushButton *BTN_ROSMODE;
    QLineEdit *OW_EDIT_T_POS;
    QLabel *OW_LABEL_1;
    QLineEdit *OW_EDIT_Y_POS;
    QLabel *OW_LABEL_3;
    QLineEdit *OW_EDIT_Z_POS;
    QLineEdit *OW_EDIT_X_POS;
    QLabel *OW_LABEL_6;
    QLabel *OW_LABEL_2;
    QPushButton *OW_GOTO_DES;
    QFrame *line_3;
    QGroupBox *groupBox_6;
    QPushButton *BTN_SW_GRASP;
    QLabel *label;
    QLineEdit *LE_SW_PZ;
    QPushButton *BTN_SW_SET;
    QPushButton *BTN_SW_READY;
    QPushButton *BTN_SW_GET_POS;
    QLineEdit *LE_SW_PX;
    QLineEdit *LE_SW_WZ;
    QPushButton *BTN_SW_BACK;
    QPushButton *BTN_SW_HOLD;
    QLabel *label_6;
    QLabel *label_5;
    QLineEdit *LE_SW_PY;
    QLabel *label_4;
    QLabel *LB_SIMTIME;

    void setupUi(QDialog *TutorialDialog)
    {
        if (TutorialDialog->objectName().isEmpty())
            TutorialDialog->setObjectName(QStringLiteral("TutorialDialog"));
        TutorialDialog->resize(690, 499);
        groupBox_3 = new QGroupBox(TutorialDialog);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        groupBox_3->setGeometry(QRect(430, 10, 251, 251));
        groupBox_3->setStyleSheet(QLatin1String("QGroupBox {\n"
"    border: 1px solid gray;\n"
"    border-radius: 5px;\n"
"    margin-top: 7px;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 8px;\n"
"    padding: 0 3px 0 3px;\n"
"}\n"
"\n"
""));
        LE_SCAN_START = new QLineEdit(groupBox_3);
        LE_SCAN_START->setObjectName(QStringLiteral("LE_SCAN_START"));
        LE_SCAN_START->setGeometry(QRect(80, 30, 71, 41));
        QFont font;
        font.setPointSize(13);
        LE_SCAN_START->setFont(font);
        LE_SCAN_START->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SCAN_END = new QLineEdit(groupBox_3);
        LE_SCAN_END->setObjectName(QStringLiteral("LE_SCAN_END"));
        LE_SCAN_END->setGeometry(QRect(80, 80, 71, 41));
        LE_SCAN_END->setFont(font);
        LE_SCAN_END->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_SCAN_TIME = new QLineEdit(groupBox_3);
        LE_SCAN_TIME->setObjectName(QStringLiteral("LE_SCAN_TIME"));
        LE_SCAN_TIME->setGeometry(QRect(80, 130, 71, 41));
        LE_SCAN_TIME->setFont(font);
        LE_SCAN_TIME->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        BTN_SCAN = new QPushButton(groupBox_3);
        BTN_SCAN->setObjectName(QStringLiteral("BTN_SCAN"));
        BTN_SCAN->setGeometry(QRect(160, 30, 81, 141));
        LE_GOTO_ANGLE = new QLineEdit(groupBox_3);
        LE_GOTO_ANGLE->setObjectName(QStringLiteral("LE_GOTO_ANGLE"));
        LE_GOTO_ANGLE->setGeometry(QRect(80, 200, 71, 41));
        LE_GOTO_ANGLE->setFont(font);
        LE_GOTO_ANGLE->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        BTN_GOTO = new QPushButton(groupBox_3);
        BTN_GOTO->setObjectName(QStringLiteral("BTN_GOTO"));
        BTN_GOTO->setGeometry(QRect(160, 200, 81, 41));
        label_7 = new QLabel(groupBox_3);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(10, 30, 61, 41));
        label_7->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_8 = new QLabel(groupBox_3);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(10, 80, 61, 41));
        label_8->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_9 = new QLabel(groupBox_3);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setGeometry(QRect(10, 130, 61, 41));
        label_9->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_10 = new QLabel(groupBox_3);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setGeometry(QRect(10, 200, 61, 41));
        label_10->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        groupBox_2 = new QGroupBox(TutorialDialog);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(140, 10, 281, 131));
        groupBox_2->setStyleSheet(QLatin1String("QGroupBox {\n"
"    border: 1px solid gray;\n"
"    border-radius: 5px;\n"
"    margin-top: 7px;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 8px;\n"
"    padding: 0 3px 0 3px;\n"
"}\n"
"\n"
""));
        BTN_WALK_READY = new QPushButton(groupBox_2);
        BTN_WALK_READY->setObjectName(QStringLiteral("BTN_WALK_READY"));
        BTN_WALK_READY->setGeometry(QRect(10, 80, 121, 41));
        line = new QFrame(groupBox_2);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(130, 30, 20, 91));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);
        BTN_TO_WHEEL = new QPushButton(groupBox_2);
        BTN_TO_WHEEL->setObjectName(QStringLiteral("BTN_TO_WHEEL"));
        BTN_TO_WHEEL->setGeometry(QRect(150, 30, 121, 41));
        BTN_TO_WALK = new QPushButton(groupBox_2);
        BTN_TO_WALK->setObjectName(QStringLiteral("BTN_TO_WALK"));
        BTN_TO_WALK->setGeometry(QRect(150, 80, 121, 41));
        BTN_HOME_POS = new QPushButton(groupBox_2);
        BTN_HOME_POS->setObjectName(QStringLiteral("BTN_HOME_POS"));
        BTN_HOME_POS->setGeometry(QRect(10, 30, 121, 41));
        groupBox_4 = new QGroupBox(TutorialDialog);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        groupBox_4->setGeometry(QRect(10, 10, 121, 131));
        groupBox_4->setStyleSheet(QLatin1String("QGroupBox {\n"
"    border: 1px solid gray;\n"
"    border-radius: 5px;\n"
"    margin-top: 7px;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 8px;\n"
"    padding: 0 3px 0 3px;\n"
"}\n"
"\n"
""));
        BTN_TEST = new QPushButton(groupBox_4);
        BTN_TEST->setObjectName(QStringLiteral("BTN_TEST"));
        BTN_TEST->setGeometry(QRect(10, 30, 101, 91));
        groupBox_5 = new QGroupBox(TutorialDialog);
        groupBox_5->setObjectName(QStringLiteral("groupBox_5"));
        groupBox_5->setGeometry(QRect(10, 160, 241, 321));
        groupBox_5->setStyleSheet(QLatin1String("QGroupBox {\n"
"    border: 1px solid gray;\n"
"    border-radius: 5px;\n"
"    margin-top: 7px;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 8px;\n"
"    padding: 0 3px 0 3px;\n"
"}\n"
"\n"
""));
        BTN_GUIMODE = new QPushButton(groupBox_5);
        BTN_GUIMODE->setObjectName(QStringLiteral("BTN_GUIMODE"));
        BTN_GUIMODE->setGeometry(QRect(10, 90, 221, 51));
        BTN_ROSMODE = new QPushButton(groupBox_5);
        BTN_ROSMODE->setObjectName(QStringLiteral("BTN_ROSMODE"));
        BTN_ROSMODE->setGeometry(QRect(10, 30, 221, 51));
        OW_EDIT_T_POS = new QLineEdit(groupBox_5);
        OW_EDIT_T_POS->setObjectName(QStringLiteral("OW_EDIT_T_POS"));
        OW_EDIT_T_POS->setGeometry(QRect(80, 270, 61, 23));
        OW_LABEL_1 = new QLabel(groupBox_5);
        OW_LABEL_1->setObjectName(QStringLiteral("OW_LABEL_1"));
        OW_LABEL_1->setGeometry(QRect(20, 180, 61, 21));
        OW_EDIT_Y_POS = new QLineEdit(groupBox_5);
        OW_EDIT_Y_POS->setObjectName(QStringLiteral("OW_EDIT_Y_POS"));
        OW_EDIT_Y_POS->setGeometry(QRect(80, 210, 61, 23));
        OW_LABEL_3 = new QLabel(groupBox_5);
        OW_LABEL_3->setObjectName(QStringLiteral("OW_LABEL_3"));
        OW_LABEL_3->setGeometry(QRect(20, 270, 61, 21));
        OW_EDIT_Z_POS = new QLineEdit(groupBox_5);
        OW_EDIT_Z_POS->setObjectName(QStringLiteral("OW_EDIT_Z_POS"));
        OW_EDIT_Z_POS->setGeometry(QRect(80, 240, 61, 23));
        OW_EDIT_X_POS = new QLineEdit(groupBox_5);
        OW_EDIT_X_POS->setObjectName(QStringLiteral("OW_EDIT_X_POS"));
        OW_EDIT_X_POS->setGeometry(QRect(80, 180, 61, 23));
        OW_LABEL_6 = new QLabel(groupBox_5);
        OW_LABEL_6->setObjectName(QStringLiteral("OW_LABEL_6"));
        OW_LABEL_6->setGeometry(QRect(20, 240, 61, 21));
        OW_LABEL_2 = new QLabel(groupBox_5);
        OW_LABEL_2->setObjectName(QStringLiteral("OW_LABEL_2"));
        OW_LABEL_2->setGeometry(QRect(20, 210, 61, 21));
        OW_GOTO_DES = new QPushButton(groupBox_5);
        OW_GOTO_DES->setObjectName(QStringLiteral("OW_GOTO_DES"));
        OW_GOTO_DES->setGeometry(QRect(150, 180, 81, 121));
        line_3 = new QFrame(groupBox_5);
        line_3->setObjectName(QStringLiteral("line_3"));
        line_3->setGeometry(QRect(20, 150, 201, 21));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);
        groupBox_6 = new QGroupBox(TutorialDialog);
        groupBox_6->setObjectName(QStringLiteral("groupBox_6"));
        groupBox_6->setGeometry(QRect(270, 270, 411, 211));
        groupBox_6->setStyleSheet(QLatin1String("QGroupBox {\n"
"    border: 1px solid gray;\n"
"    border-radius: 5px;\n"
"    margin-top: 7px;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 8px;\n"
"    padding: 0 3px 0 3px;\n"
"}\n"
"\n"
""));
        BTN_SW_GRASP = new QPushButton(groupBox_6);
        BTN_SW_GRASP->setObjectName(QStringLiteral("BTN_SW_GRASP"));
        BTN_SW_GRASP->setGeometry(QRect(300, 30, 99, 41));
        label = new QLabel(groupBox_6);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(220, 30, 67, 31));
        label->setAlignment(Qt::AlignCenter);
        LE_SW_PZ = new QLineEdit(groupBox_6);
        LE_SW_PZ->setObjectName(QStringLiteral("LE_SW_PZ"));
        LE_SW_PZ->setGeometry(QRect(130, 90, 81, 27));
        LE_SW_PZ->setAlignment(Qt::AlignCenter);
        BTN_SW_SET = new QPushButton(groupBox_6);
        BTN_SW_SET->setObjectName(QStringLiteral("BTN_SW_SET"));
        BTN_SW_SET->setGeometry(QRect(130, 166, 151, 31));
        BTN_SW_READY = new QPushButton(groupBox_6);
        BTN_SW_READY->setObjectName(QStringLiteral("BTN_SW_READY"));
        BTN_SW_READY->setGeometry(QRect(20, 30, 91, 71));
        BTN_SW_GET_POS = new QPushButton(groupBox_6);
        BTN_SW_GET_POS->setObjectName(QStringLiteral("BTN_SW_GET_POS"));
        BTN_SW_GET_POS->setGeometry(QRect(20, 120, 91, 71));
        LE_SW_PX = new QLineEdit(groupBox_6);
        LE_SW_PX->setObjectName(QStringLiteral("LE_SW_PX"));
        LE_SW_PX->setGeometry(QRect(130, 30, 81, 27));
        LE_SW_PX->setAlignment(Qt::AlignCenter);
        LE_SW_WZ = new QLineEdit(groupBox_6);
        LE_SW_WZ->setObjectName(QStringLiteral("LE_SW_WZ"));
        LE_SW_WZ->setGeometry(QRect(130, 130, 81, 27));
        LE_SW_WZ->setAlignment(Qt::AlignCenter);
        BTN_SW_BACK = new QPushButton(groupBox_6);
        BTN_SW_BACK->setObjectName(QStringLiteral("BTN_SW_BACK"));
        BTN_SW_BACK->setGeometry(QRect(300, 130, 99, 41));
        BTN_SW_HOLD = new QPushButton(groupBox_6);
        BTN_SW_HOLD->setObjectName(QStringLiteral("BTN_SW_HOLD"));
        BTN_SW_HOLD->setGeometry(QRect(300, 80, 99, 41));
        label_6 = new QLabel(groupBox_6);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(220, 130, 67, 31));
        label_6->setAlignment(Qt::AlignCenter);
        label_5 = new QLabel(groupBox_6);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(220, 90, 67, 31));
        label_5->setAlignment(Qt::AlignCenter);
        LE_SW_PY = new QLineEdit(groupBox_6);
        LE_SW_PY->setObjectName(QStringLiteral("LE_SW_PY"));
        LE_SW_PY->setGeometry(QRect(130, 60, 81, 27));
        LE_SW_PY->setAlignment(Qt::AlignCenter);
        label_4 = new QLabel(groupBox_6);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(220, 60, 67, 31));
        label_4->setAlignment(Qt::AlignCenter);
        LB_SIMTIME = new QLabel(TutorialDialog);
        LB_SIMTIME->setObjectName(QStringLiteral("LB_SIMTIME"));
        LB_SIMTIME->setGeometry(QRect(270, 160, 151, 91));
        QFont font1;
        font1.setPointSize(12);
        LB_SIMTIME->setFont(font1);

        retranslateUi(TutorialDialog);

        QMetaObject::connectSlotsByName(TutorialDialog);
    } // setupUi

    void retranslateUi(QDialog *TutorialDialog)
    {
        TutorialDialog->setWindowTitle(QApplication::translate("TutorialDialog", "Dialog", 0));
        groupBox_3->setTitle(QApplication::translate("TutorialDialog", "Head Control", 0));
        LE_SCAN_START->setText(QApplication::translate("TutorialDialog", "-30", 0));
        LE_SCAN_END->setText(QApplication::translate("TutorialDialog", "30", 0));
        LE_SCAN_TIME->setText(QApplication::translate("TutorialDialog", "2000", 0));
        BTN_SCAN->setText(QApplication::translate("TutorialDialog", "SCAN", 0));
        LE_GOTO_ANGLE->setText(QApplication::translate("TutorialDialog", "0", 0));
        BTN_GOTO->setText(QApplication::translate("TutorialDialog", "GO TO", 0));
        label_7->setText(QApplication::translate("TutorialDialog", "FROM", 0));
        label_8->setText(QApplication::translate("TutorialDialog", "TO", 0));
        label_9->setText(QApplication::translate("TutorialDialog", "TIME", 0));
        label_10->setText(QApplication::translate("TutorialDialog", "TARGET", 0));
        groupBox_2->setTitle(QApplication::translate("TutorialDialog", "Pose Change", 0));
        BTN_WALK_READY->setText(QApplication::translate("TutorialDialog", "Walk Ready", 0));
        BTN_TO_WHEEL->setText(QApplication::translate("TutorialDialog", "To Wheel", 0));
        BTN_TO_WALK->setText(QApplication::translate("TutorialDialog", "To Walk", 0));
        BTN_HOME_POS->setText(QApplication::translate("TutorialDialog", "Home Pos", 0));
        groupBox_4->setTitle(QApplication::translate("TutorialDialog", "Tutorial", 0));
        BTN_TEST->setText(QApplication::translate("TutorialDialog", "Test", 0));
        groupBox_5->setTitle(QApplication::translate("TutorialDialog", "Wheel Control", 0));
        BTN_GUIMODE->setText(QApplication::translate("TutorialDialog", "AL Mode", 0));
        BTN_ROSMODE->setText(QApplication::translate("TutorialDialog", "ROS Mode", 0));
        OW_EDIT_T_POS->setText(QApplication::translate("TutorialDialog", "0", 0));
        OW_LABEL_1->setText(QApplication::translate("TutorialDialog", "X-pos(m)", 0));
        OW_EDIT_Y_POS->setText(QApplication::translate("TutorialDialog", "0", 0));
        OW_LABEL_3->setText(QApplication::translate("TutorialDialog", "Angle(')", 0));
        OW_EDIT_Z_POS->setText(QApplication::translate("TutorialDialog", "0", 0));
        OW_EDIT_X_POS->setText(QApplication::translate("TutorialDialog", "0", 0));
        OW_LABEL_6->setText(QApplication::translate("TutorialDialog", "Z-pos(m)", 0));
        OW_LABEL_2->setText(QApplication::translate("TutorialDialog", "Y-pos(m)", 0));
        OW_GOTO_DES->setText(QApplication::translate("TutorialDialog", "Goto\n"
"Point", 0));
        groupBox_6->setTitle(QApplication::translate("TutorialDialog", "Object Picking", 0));
        BTN_SW_GRASP->setText(QApplication::translate("TutorialDialog", "Grasp", 0));
        label->setText(QApplication::translate("TutorialDialog", "Px (m)", 0));
        LE_SW_PZ->setText(QApplication::translate("TutorialDialog", "0.42", 0));
        BTN_SW_SET->setText(QApplication::translate("TutorialDialog", "Set", 0));
        BTN_SW_READY->setText(QApplication::translate("TutorialDialog", "Ready", 0));
        BTN_SW_GET_POS->setText(QApplication::translate("TutorialDialog", "Get Pos", 0));
        LE_SW_PX->setText(QApplication::translate("TutorialDialog", "0.65", 0));
        LE_SW_WZ->setText(QApplication::translate("TutorialDialog", "30", 0));
        BTN_SW_BACK->setText(QApplication::translate("TutorialDialog", "Back", 0));
        BTN_SW_HOLD->setText(QApplication::translate("TutorialDialog", "Hold", 0));
        label_6->setText(QApplication::translate("TutorialDialog", "Wx (deg)", 0));
        label_5->setText(QApplication::translate("TutorialDialog", "Pz (m)", 0));
        LE_SW_PY->setText(QApplication::translate("TutorialDialog", "-0.30", 0));
        label_4->setText(QApplication::translate("TutorialDialog", "Py (m)", 0));
        LB_SIMTIME->setText(QApplication::translate("TutorialDialog", "SimTime: \n"
"", 0));
    } // retranslateUi

};

namespace Ui {
    class TutorialDialog: public Ui_TutorialDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TUTORIALDIALOG_H
