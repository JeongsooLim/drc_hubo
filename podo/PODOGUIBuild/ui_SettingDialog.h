/********************************************************************************
** Form generated from reading UI file 'SettingDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SETTINGDIALOG_H
#define UI_SETTINGDIALOG_H

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
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QTableWidget>

QT_BEGIN_NAMESPACE

class Ui_SettingDialog
{
public:
    QTableWidget *TW_0;
    QTableWidget *TW_1;
    QTableWidget *TW_2;
    QTableWidget *TW_3;
    QLabel *LB_SELECTED;
    QPushButton *BTN_CAN_CHECK;
    QPushButton *BTN_FIND_HOME;
    QGroupBox *groupBox_2;
    QPushButton *BTN_MOVE_JOINT;
    QLabel *label_2;
    QLineEdit *LE_MOVE_DEGREE;
    QLineEdit *LE_MOVE_TIME;
    QLabel *label;
    QPushButton *BTN_GAIN_OVERRIDE;
    QLineEdit *LE_GO_GAIN;
    QLabel *label_3;
    QLineEdit *LE_GO_TIME;
    QLabel *label_4;
    QFrame *line_2;
    QFrame *line;
    QGroupBox *groupBox;
    QPushButton *BTN_EXECUTE_COMMAND;
    QRadioButton *RB_MODE_POS;
    QRadioButton *RB_MODE_PWM;
    QRadioButton *RB_FRIC_OFF;
    QRadioButton *RB_FRIC_ON;
    QRadioButton *RB_SW_COMP;
    QRadioButton *RB_SW_NON_COMP;
    QRadioButton *RB_INIT_POS;
    QRadioButton *RB_ENC_ZERO;
    QRadioButton *RB_FET_OFF;
    QRadioButton *RB_FET_ON;
    QRadioButton *RB_CTRL_OFF;
    QRadioButton *RB_CTRL_ON;
    QFrame *line_3;
    QFrame *line_4;
    QFrame *line_5;
    QRadioButton *RB_ERROR_CLEAR;
    QRadioButton *RB_JOINT_RECOVER;

    void setupUi(QDialog *SettingDialog)
    {
        if (SettingDialog->objectName().isEmpty())
            SettingDialog->setObjectName(QStringLiteral("SettingDialog"));
        SettingDialog->resize(680, 570);
        TW_0 = new QTableWidget(SettingDialog);
        TW_0->setObjectName(QStringLiteral("TW_0"));
        TW_0->setGeometry(QRect(10, 10, 221, 240));
        TW_1 = new QTableWidget(SettingDialog);
        TW_1->setObjectName(QStringLiteral("TW_1"));
        TW_1->setGeometry(QRect(10, 290, 221, 271));
        TW_2 = new QTableWidget(SettingDialog);
        TW_2->setObjectName(QStringLiteral("TW_2"));
        TW_2->setGeometry(QRect(240, 10, 221, 271));
        TW_2->setSelectionMode(QAbstractItemView::SingleSelection);
        TW_2->setSelectionBehavior(QAbstractItemView::SelectRows);
        TW_3 = new QTableWidget(SettingDialog);
        TW_3->setObjectName(QStringLiteral("TW_3"));
        TW_3->setGeometry(QRect(240, 290, 221, 271));
        LB_SELECTED = new QLabel(SettingDialog);
        LB_SELECTED->setObjectName(QStringLiteral("LB_SELECTED"));
        LB_SELECTED->setGeometry(QRect(480, 60, 121, 21));
        BTN_CAN_CHECK = new QPushButton(SettingDialog);
        BTN_CAN_CHECK->setObjectName(QStringLiteral("BTN_CAN_CHECK"));
        BTN_CAN_CHECK->setGeometry(QRect(470, 10, 91, 31));
        QFont font;
        font.setPointSize(10);
        BTN_CAN_CHECK->setFont(font);
        BTN_FIND_HOME = new QPushButton(SettingDialog);
        BTN_FIND_HOME->setObjectName(QStringLiteral("BTN_FIND_HOME"));
        BTN_FIND_HOME->setGeometry(QRect(580, 10, 91, 31));
        BTN_FIND_HOME->setFont(font);
        groupBox_2 = new QGroupBox(SettingDialog);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(470, 90, 201, 161));
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
        BTN_MOVE_JOINT = new QPushButton(groupBox_2);
        BTN_MOVE_JOINT->setObjectName(QStringLiteral("BTN_MOVE_JOINT"));
        BTN_MOVE_JOINT->setGeometry(QRect(130, 20, 61, 51));
        BTN_MOVE_JOINT->setFont(font);
        label_2 = new QLabel(groupBox_2);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(10, 50, 41, 21));
        label_2->setFont(font);
        label_2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_MOVE_DEGREE = new QLineEdit(groupBox_2);
        LE_MOVE_DEGREE->setObjectName(QStringLiteral("LE_MOVE_DEGREE"));
        LE_MOVE_DEGREE->setGeometry(QRect(60, 20, 61, 23));
        LE_MOVE_DEGREE->setFont(font);
        LE_MOVE_DEGREE->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_MOVE_TIME = new QLineEdit(groupBox_2);
        LE_MOVE_TIME->setObjectName(QStringLiteral("LE_MOVE_TIME"));
        LE_MOVE_TIME->setGeometry(QRect(60, 50, 61, 23));
        LE_MOVE_TIME->setFont(font);
        LE_MOVE_TIME->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label = new QLabel(groupBox_2);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(10, 20, 41, 21));
        label->setFont(font);
        label->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        BTN_GAIN_OVERRIDE = new QPushButton(groupBox_2);
        BTN_GAIN_OVERRIDE->setObjectName(QStringLiteral("BTN_GAIN_OVERRIDE"));
        BTN_GAIN_OVERRIDE->setGeometry(QRect(130, 100, 61, 51));
        BTN_GAIN_OVERRIDE->setFont(font);
        LE_GO_GAIN = new QLineEdit(groupBox_2);
        LE_GO_GAIN->setObjectName(QStringLiteral("LE_GO_GAIN"));
        LE_GO_GAIN->setGeometry(QRect(60, 100, 61, 23));
        LE_GO_GAIN->setFont(font);
        LE_GO_GAIN->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_3 = new QLabel(groupBox_2);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(10, 100, 41, 21));
        label_3->setFont(font);
        label_3->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_GO_TIME = new QLineEdit(groupBox_2);
        LE_GO_TIME->setObjectName(QStringLiteral("LE_GO_TIME"));
        LE_GO_TIME->setGeometry(QRect(60, 130, 61, 23));
        LE_GO_TIME->setFont(font);
        LE_GO_TIME->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_4 = new QLabel(groupBox_2);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(10, 130, 41, 21));
        label_4->setFont(font);
        label_4->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        line_2 = new QFrame(groupBox_2);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setGeometry(QRect(10, 80, 91, 16));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        line = new QFrame(SettingDialog);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(470, 43, 111, 16));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        groupBox = new QGroupBox(SettingDialog);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(470, 260, 201, 301));
        groupBox->setStyleSheet(QLatin1String("QGroupBox {\n"
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
        BTN_EXECUTE_COMMAND = new QPushButton(groupBox);
        BTN_EXECUTE_COMMAND->setObjectName(QStringLiteral("BTN_EXECUTE_COMMAND"));
        BTN_EXECUTE_COMMAND->setGeometry(QRect(10, 260, 181, 31));
        BTN_EXECUTE_COMMAND->setFont(font);
        RB_MODE_POS = new QRadioButton(groupBox);
        RB_MODE_POS->setObjectName(QStringLiteral("RB_MODE_POS"));
        RB_MODE_POS->setGeometry(QRect(10, 200, 91, 22));
        RB_MODE_POS->setFont(font);
        RB_MODE_PWM = new QRadioButton(groupBox);
        RB_MODE_PWM->setObjectName(QStringLiteral("RB_MODE_PWM"));
        RB_MODE_PWM->setGeometry(QRect(100, 200, 91, 22));
        RB_MODE_PWM->setFont(font);
        RB_FRIC_OFF = new QRadioButton(groupBox);
        RB_FRIC_OFF->setObjectName(QStringLiteral("RB_FRIC_OFF"));
        RB_FRIC_OFF->setGeometry(QRect(100, 160, 91, 22));
        RB_FRIC_OFF->setFont(font);
        RB_FRIC_ON = new QRadioButton(groupBox);
        RB_FRIC_ON->setObjectName(QStringLiteral("RB_FRIC_ON"));
        RB_FRIC_ON->setGeometry(QRect(10, 160, 91, 22));
        RB_FRIC_ON->setFont(font);
        RB_SW_COMP = new QRadioButton(groupBox);
        RB_SW_COMP->setObjectName(QStringLiteral("RB_SW_COMP"));
        RB_SW_COMP->setGeometry(QRect(10, 130, 91, 22));
        RB_SW_COMP->setFont(font);
        RB_SW_NON_COMP = new QRadioButton(groupBox);
        RB_SW_NON_COMP->setObjectName(QStringLiteral("RB_SW_NON_COMP"));
        RB_SW_NON_COMP->setGeometry(QRect(100, 130, 91, 22));
        RB_SW_NON_COMP->setFont(font);
        RB_INIT_POS = new QRadioButton(groupBox);
        RB_INIT_POS->setObjectName(QStringLiteral("RB_INIT_POS"));
        RB_INIT_POS->setGeometry(QRect(10, 20, 91, 22));
        RB_INIT_POS->setFont(font);
        RB_INIT_POS->setChecked(true);
        RB_ENC_ZERO = new QRadioButton(groupBox);
        RB_ENC_ZERO->setObjectName(QStringLiteral("RB_ENC_ZERO"));
        RB_ENC_ZERO->setGeometry(QRect(100, 20, 91, 22));
        RB_ENC_ZERO->setFont(font);
        RB_FET_OFF = new QRadioButton(groupBox);
        RB_FET_OFF->setObjectName(QStringLiteral("RB_FET_OFF"));
        RB_FET_OFF->setGeometry(QRect(100, 60, 91, 22));
        RB_FET_OFF->setFont(font);
        RB_FET_ON = new QRadioButton(groupBox);
        RB_FET_ON->setObjectName(QStringLiteral("RB_FET_ON"));
        RB_FET_ON->setGeometry(QRect(10, 60, 91, 22));
        RB_FET_ON->setFont(font);
        RB_CTRL_OFF = new QRadioButton(groupBox);
        RB_CTRL_OFF->setObjectName(QStringLiteral("RB_CTRL_OFF"));
        RB_CTRL_OFF->setGeometry(QRect(100, 90, 91, 22));
        RB_CTRL_OFF->setFont(font);
        RB_CTRL_ON = new QRadioButton(groupBox);
        RB_CTRL_ON->setObjectName(QStringLiteral("RB_CTRL_ON"));
        RB_CTRL_ON->setGeometry(QRect(10, 90, 91, 22));
        RB_CTRL_ON->setFont(font);
        line_3 = new QFrame(groupBox);
        line_3->setObjectName(QStringLiteral("line_3"));
        line_3->setGeometry(QRect(10, 44, 91, 16));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);
        line_4 = new QFrame(groupBox);
        line_4->setObjectName(QStringLiteral("line_4"));
        line_4->setGeometry(QRect(10, 114, 91, 16));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);
        line_5 = new QFrame(groupBox);
        line_5->setObjectName(QStringLiteral("line_5"));
        line_5->setGeometry(QRect(10, 184, 91, 16));
        line_5->setFrameShape(QFrame::HLine);
        line_5->setFrameShadow(QFrame::Sunken);
        RB_ERROR_CLEAR = new QRadioButton(groupBox);
        RB_ERROR_CLEAR->setObjectName(QStringLiteral("RB_ERROR_CLEAR"));
        RB_ERROR_CLEAR->setGeometry(QRect(10, 230, 91, 22));
        RB_ERROR_CLEAR->setFont(font);
        RB_JOINT_RECOVER = new QRadioButton(groupBox);
        RB_JOINT_RECOVER->setObjectName(QStringLiteral("RB_JOINT_RECOVER"));
        RB_JOINT_RECOVER->setGeometry(QRect(100, 230, 91, 22));
        RB_JOINT_RECOVER->setFont(font);
        BTN_EXECUTE_COMMAND->raise();
        TW_3->raise();
        RB_MODE_POS->raise();
        RB_MODE_PWM->raise();
        RB_FRIC_OFF->raise();
        RB_FRIC_ON->raise();
        RB_SW_COMP->raise();
        RB_SW_NON_COMP->raise();
        RB_INIT_POS->raise();
        RB_ENC_ZERO->raise();
        RB_FET_OFF->raise();
        RB_FET_ON->raise();
        RB_CTRL_OFF->raise();
        RB_CTRL_ON->raise();
        line_3->raise();
        line_4->raise();
        line_5->raise();
        RB_ERROR_CLEAR->raise();
        RB_JOINT_RECOVER->raise();

        retranslateUi(SettingDialog);

        QMetaObject::connectSlotsByName(SettingDialog);
    } // setupUi

    void retranslateUi(QDialog *SettingDialog)
    {
        SettingDialog->setWindowTitle(QApplication::translate("SettingDialog", "Dialog", 0));
        LB_SELECTED->setText(QApplication::translate("SettingDialog", "Selected: ", 0));
        BTN_CAN_CHECK->setText(QApplication::translate("SettingDialog", "CAN Check", 0));
        BTN_FIND_HOME->setText(QApplication::translate("SettingDialog", "Find Home", 0));
        groupBox_2->setTitle(QApplication::translate("SettingDialog", "Move", 0));
        BTN_MOVE_JOINT->setText(QApplication::translate("SettingDialog", "Move\n"
"Joint", 0));
        label_2->setText(QApplication::translate("SettingDialog", "Time :", 0));
        LE_MOVE_DEGREE->setText(QApplication::translate("SettingDialog", "10", 0));
        LE_MOVE_TIME->setText(QApplication::translate("SettingDialog", "2000", 0));
        label->setText(QApplication::translate("SettingDialog", "Angle :", 0));
        BTN_GAIN_OVERRIDE->setText(QApplication::translate("SettingDialog", "Gain\n"
"Override", 0));
        LE_GO_GAIN->setText(QApplication::translate("SettingDialog", "10", 0));
        label_3->setText(QApplication::translate("SettingDialog", "Gain :", 0));
        LE_GO_TIME->setText(QApplication::translate("SettingDialog", "2000", 0));
        label_4->setText(QApplication::translate("SettingDialog", "Time :", 0));
        groupBox->setTitle(QApplication::translate("SettingDialog", "Control", 0));
        BTN_EXECUTE_COMMAND->setText(QApplication::translate("SettingDialog", "Execute", 0));
        RB_MODE_POS->setText(QApplication::translate("SettingDialog", "Position", 0));
        RB_MODE_PWM->setText(QApplication::translate("SettingDialog", "PWM", 0));
        RB_FRIC_OFF->setText(QApplication::translate("SettingDialog", "Fric Off", 0));
        RB_FRIC_ON->setText(QApplication::translate("SettingDialog", "Fric On", 0));
        RB_SW_COMP->setText(QApplication::translate("SettingDialog", "Comp.", 0));
        RB_SW_NON_COMP->setText(QApplication::translate("SettingDialog", "N-Comp.", 0));
        RB_INIT_POS->setText(QApplication::translate("SettingDialog", "Init Pos", 0));
        RB_ENC_ZERO->setText(QApplication::translate("SettingDialog", "Enc Zero", 0));
        RB_FET_OFF->setText(QApplication::translate("SettingDialog", "FET Off", 0));
        RB_FET_ON->setText(QApplication::translate("SettingDialog", "FET On", 0));
        RB_CTRL_OFF->setText(QApplication::translate("SettingDialog", "Ctrl Off", 0));
        RB_CTRL_ON->setText(QApplication::translate("SettingDialog", "Ctrl On", 0));
        RB_ERROR_CLEAR->setText(QApplication::translate("SettingDialog", "Err Clear", 0));
        RB_JOINT_RECOVER->setText(QApplication::translate("SettingDialog", "Recover", 0));
    } // retranslateUi

};

namespace Ui {
    class SettingDialog: public Ui_SettingDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SETTINGDIALOG_H
