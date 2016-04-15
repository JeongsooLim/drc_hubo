/********************************************************************************
** Form generated from reading UI file 'JointDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_JOINTDIALOG_H
#define UI_JOINTDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_JointDialog
{
public:
    QLabel *label;
    QGroupBox *groupBox;
    QWidget *gridLayoutWidget_3;
    QGridLayout *gridLayout_3;
    QLineEdit *LE_JOINT_WST;
    QLabel *label_18;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QLineEdit *LE_JOINT_RWP;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *label_3;
    QLineEdit *LE_JOINT_RSP;
    QLabel *label_2;
    QLabel *label_4;
    QLineEdit *LE_JOINT_RSY;
    QLineEdit *LE_JOINT_RSR;
    QLineEdit *LE_JOINT_REB;
    QLabel *label_7;
    QLineEdit *LE_JOINT_RWY;
    QLabel *label_81;
    QLineEdit *LE_JOINT_RWY2;
    QLabel *label_79;
    QLineEdit *LE_JOINT_RHAND;
    QWidget *horizontalLayoutWidget_2;
    QHBoxLayout *horizontalLayout_2;
    QRadioButton *RB_JOINT_REFERENCE;
    QRadioButton *RB_JOINT_ENCODER;
    QWidget *gridLayoutWidget_2;
    QGridLayout *gridLayout_2;
    QLabel *label_8;
    QLabel *label_9;
    QLineEdit *LE_JOINT_LSP;
    QLabel *label_10;
    QLabel *label_11;
    QLineEdit *LE_JOINT_LSR;
    QLabel *label_12;
    QLabel *label_13;
    QLineEdit *LE_JOINT_LSY;
    QLineEdit *LE_JOINT_LEB;
    QLineEdit *LE_JOINT_LWY;
    QLineEdit *LE_JOINT_LWP;
    QLineEdit *LE_JOINT_LHAND;
    QLabel *label_80;
    QLabel *label_83;
    QLineEdit *LE_JOINT_LWY2;
    QWidget *gridLayoutWidget_4;
    QGridLayout *gridLayout_4;
    QLineEdit *LE_JOINT_RAR;
    QLabel *label_14;
    QLabel *label_19;
    QLabel *label_20;
    QLineEdit *LE_JOINT_RHY;
    QLabel *label_21;
    QLabel *label_22;
    QLineEdit *LE_JOINT_RHP;
    QLineEdit *LE_JOINT_RHR;
    QLineEdit *LE_JOINT_RKN;
    QLabel *label_23;
    QLineEdit *LE_JOINT_RAP;
    QWidget *gridLayoutWidget_5;
    QGridLayout *gridLayout_6;
    QLineEdit *LE_JOINT_LAR;
    QLabel *label_30;
    QLabel *label_31;
    QLabel *label_32;
    QLineEdit *LE_JOINT_LHY;
    QLabel *label_33;
    QLabel *label_34;
    QLineEdit *LE_JOINT_LHP;
    QLineEdit *LE_JOINT_LHR;
    QLineEdit *LE_JOINT_LKN;
    QLabel *label_35;
    QLineEdit *LE_JOINT_LAP;
    QWidget *gridLayoutWidget_6;
    QGridLayout *gridLayout_5;
    QLabel *label_27;
    QLabel *label_25;
    QLineEdit *LE_JOINT_LWH;
    QLineEdit *LE_JOINT_RWH;
    QPushButton *BTN_ENC_ENABLE;
    QPushButton *BTN_ENC_DISABLE;
    QFrame *line;

    void setupUi(QDialog *JointDialog)
    {
        if (JointDialog->objectName().isEmpty())
            JointDialog->setObjectName(QStringLiteral("JointDialog"));
        JointDialog->resize(320, 620);
        label = new QLabel(JointDialog);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(10, 0, 111, 31));
        QFont font;
        font.setPointSize(12);
        font.setBold(true);
        font.setWeight(75);
        label->setFont(font);
        groupBox = new QGroupBox(JointDialog);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(10, 50, 301, 561));
        QFont font1;
        font1.setBold(true);
        font1.setWeight(75);
        groupBox->setFont(font1);
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
        gridLayoutWidget_3 = new QWidget(groupBox);
        gridLayoutWidget_3->setObjectName(QStringLiteral("gridLayoutWidget_3"));
        gridLayoutWidget_3->setGeometry(QRect(160, 500, 128, 31));
        gridLayout_3 = new QGridLayout(gridLayoutWidget_3);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        gridLayout_3->setContentsMargins(0, 0, 0, 0);
        LE_JOINT_WST = new QLineEdit(gridLayoutWidget_3);
        LE_JOINT_WST->setObjectName(QStringLiteral("LE_JOINT_WST"));
        LE_JOINT_WST->setEnabled(false);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(LE_JOINT_WST->sizePolicy().hasHeightForWidth());
        LE_JOINT_WST->setSizePolicy(sizePolicy);
        LE_JOINT_WST->setMinimumSize(QSize(70, 20));
        LE_JOINT_WST->setMaximumSize(QSize(70, 20));
        QFont font2;
        font2.setFamily(QStringLiteral("Serif"));
        font2.setPointSize(9);
        LE_JOINT_WST->setFont(font2);
        LE_JOINT_WST->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_WST->setReadOnly(true);

        gridLayout_3->addWidget(LE_JOINT_WST, 0, 1, 1, 1);

        label_18 = new QLabel(gridLayoutWidget_3);
        label_18->setObjectName(QStringLiteral("label_18"));
        sizePolicy.setHeightForWidth(label_18->sizePolicy().hasHeightForWidth());
        label_18->setSizePolicy(sizePolicy);
        label_18->setMinimumSize(QSize(50, 20));
        label_18->setMaximumSize(QSize(50, 20));
        label_18->setFont(font2);
        label_18->setAlignment(Qt::AlignCenter);

        gridLayout_3->addWidget(label_18, 0, 0, 1, 1);

        gridLayoutWidget = new QWidget(groupBox);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(10, 80, 131, 211));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        LE_JOINT_RWP = new QLineEdit(gridLayoutWidget);
        LE_JOINT_RWP->setObjectName(QStringLiteral("LE_JOINT_RWP"));
        LE_JOINT_RWP->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_RWP->sizePolicy().hasHeightForWidth());
        LE_JOINT_RWP->setSizePolicy(sizePolicy);
        LE_JOINT_RWP->setMinimumSize(QSize(70, 20));
        LE_JOINT_RWP->setMaximumSize(QSize(70, 20));
        LE_JOINT_RWP->setFont(font2);
        LE_JOINT_RWP->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RWP->setReadOnly(true);

        gridLayout->addWidget(LE_JOINT_RWP, 5, 1, 1, 1);

        label_5 = new QLabel(gridLayoutWidget);
        label_5->setObjectName(QStringLiteral("label_5"));
        sizePolicy.setHeightForWidth(label_5->sizePolicy().hasHeightForWidth());
        label_5->setSizePolicy(sizePolicy);
        label_5->setMinimumSize(QSize(50, 20));
        label_5->setMaximumSize(QSize(50, 20));
        label_5->setFont(font2);
        label_5->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_5, 4, 0, 1, 1);

        label_6 = new QLabel(gridLayoutWidget);
        label_6->setObjectName(QStringLiteral("label_6"));
        sizePolicy.setHeightForWidth(label_6->sizePolicy().hasHeightForWidth());
        label_6->setSizePolicy(sizePolicy);
        label_6->setMinimumSize(QSize(50, 20));
        label_6->setMaximumSize(QSize(50, 20));
        label_6->setFont(font2);
        label_6->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_6, 5, 0, 1, 1);

        label_3 = new QLabel(gridLayoutWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        sizePolicy.setHeightForWidth(label_3->sizePolicy().hasHeightForWidth());
        label_3->setSizePolicy(sizePolicy);
        label_3->setMinimumSize(QSize(50, 20));
        label_3->setMaximumSize(QSize(50, 20));
        label_3->setFont(font2);
        label_3->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_3, 2, 0, 1, 1);

        LE_JOINT_RSP = new QLineEdit(gridLayoutWidget);
        LE_JOINT_RSP->setObjectName(QStringLiteral("LE_JOINT_RSP"));
        LE_JOINT_RSP->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_RSP->sizePolicy().hasHeightForWidth());
        LE_JOINT_RSP->setSizePolicy(sizePolicy);
        LE_JOINT_RSP->setMinimumSize(QSize(70, 20));
        LE_JOINT_RSP->setMaximumSize(QSize(70, 20));
        QFont font3;
        font3.setFamily(QStringLiteral("Serif"));
        font3.setPointSize(8);
        LE_JOINT_RSP->setFont(font3);
        LE_JOINT_RSP->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RSP->setReadOnly(true);

        gridLayout->addWidget(LE_JOINT_RSP, 0, 1, 1, 1);

        label_2 = new QLabel(gridLayoutWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        sizePolicy.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy);
        label_2->setMinimumSize(QSize(50, 20));
        label_2->setMaximumSize(QSize(50, 20));
        label_2->setFont(font2);
        label_2->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_2, 0, 0, 1, 1);

        label_4 = new QLabel(gridLayoutWidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        sizePolicy.setHeightForWidth(label_4->sizePolicy().hasHeightForWidth());
        label_4->setSizePolicy(sizePolicy);
        label_4->setMinimumSize(QSize(50, 20));
        label_4->setMaximumSize(QSize(50, 20));
        label_4->setFont(font2);
        label_4->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_4, 3, 0, 1, 1);

        LE_JOINT_RSY = new QLineEdit(gridLayoutWidget);
        LE_JOINT_RSY->setObjectName(QStringLiteral("LE_JOINT_RSY"));
        LE_JOINT_RSY->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_RSY->sizePolicy().hasHeightForWidth());
        LE_JOINT_RSY->setSizePolicy(sizePolicy);
        LE_JOINT_RSY->setMinimumSize(QSize(70, 20));
        LE_JOINT_RSY->setMaximumSize(QSize(70, 20));
        LE_JOINT_RSY->setFont(font3);
        LE_JOINT_RSY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RSY->setReadOnly(true);

        gridLayout->addWidget(LE_JOINT_RSY, 2, 1, 1, 1);

        LE_JOINT_RSR = new QLineEdit(gridLayoutWidget);
        LE_JOINT_RSR->setObjectName(QStringLiteral("LE_JOINT_RSR"));
        LE_JOINT_RSR->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_RSR->sizePolicy().hasHeightForWidth());
        LE_JOINT_RSR->setSizePolicy(sizePolicy);
        LE_JOINT_RSR->setMinimumSize(QSize(70, 20));
        LE_JOINT_RSR->setMaximumSize(QSize(70, 20));
        LE_JOINT_RSR->setFont(font2);
        LE_JOINT_RSR->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RSR->setReadOnly(true);

        gridLayout->addWidget(LE_JOINT_RSR, 1, 1, 1, 1);

        LE_JOINT_REB = new QLineEdit(gridLayoutWidget);
        LE_JOINT_REB->setObjectName(QStringLiteral("LE_JOINT_REB"));
        LE_JOINT_REB->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_REB->sizePolicy().hasHeightForWidth());
        LE_JOINT_REB->setSizePolicy(sizePolicy);
        LE_JOINT_REB->setMinimumSize(QSize(70, 20));
        LE_JOINT_REB->setMaximumSize(QSize(70, 20));
        LE_JOINT_REB->setFont(font2);
        LE_JOINT_REB->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_REB->setReadOnly(true);

        gridLayout->addWidget(LE_JOINT_REB, 3, 1, 1, 1);

        label_7 = new QLabel(gridLayoutWidget);
        label_7->setObjectName(QStringLiteral("label_7"));
        sizePolicy.setHeightForWidth(label_7->sizePolicy().hasHeightForWidth());
        label_7->setSizePolicy(sizePolicy);
        label_7->setMinimumSize(QSize(50, 20));
        label_7->setMaximumSize(QSize(50, 20));
        label_7->setFont(font3);
        label_7->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_7, 1, 0, 1, 1);

        LE_JOINT_RWY = new QLineEdit(gridLayoutWidget);
        LE_JOINT_RWY->setObjectName(QStringLiteral("LE_JOINT_RWY"));
        LE_JOINT_RWY->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_RWY->sizePolicy().hasHeightForWidth());
        LE_JOINT_RWY->setSizePolicy(sizePolicy);
        LE_JOINT_RWY->setMinimumSize(QSize(70, 20));
        LE_JOINT_RWY->setMaximumSize(QSize(70, 20));
        LE_JOINT_RWY->setFont(font2);
        LE_JOINT_RWY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RWY->setReadOnly(true);

        gridLayout->addWidget(LE_JOINT_RWY, 4, 1, 1, 1);

        label_81 = new QLabel(gridLayoutWidget);
        label_81->setObjectName(QStringLiteral("label_81"));
        sizePolicy.setHeightForWidth(label_81->sizePolicy().hasHeightForWidth());
        label_81->setSizePolicy(sizePolicy);
        label_81->setMinimumSize(QSize(50, 20));
        label_81->setMaximumSize(QSize(50, 20));
        label_81->setFont(font2);
        label_81->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_81, 6, 0, 1, 1);

        LE_JOINT_RWY2 = new QLineEdit(gridLayoutWidget);
        LE_JOINT_RWY2->setObjectName(QStringLiteral("LE_JOINT_RWY2"));
        LE_JOINT_RWY2->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_RWY2->sizePolicy().hasHeightForWidth());
        LE_JOINT_RWY2->setSizePolicy(sizePolicy);
        LE_JOINT_RWY2->setMinimumSize(QSize(70, 20));
        LE_JOINT_RWY2->setMaximumSize(QSize(70, 20));
        LE_JOINT_RWY2->setFont(font3);
        LE_JOINT_RWY2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RWY2->setReadOnly(true);

        gridLayout->addWidget(LE_JOINT_RWY2, 6, 1, 1, 1);

        label_79 = new QLabel(gridLayoutWidget);
        label_79->setObjectName(QStringLiteral("label_79"));
        sizePolicy.setHeightForWidth(label_79->sizePolicy().hasHeightForWidth());
        label_79->setSizePolicy(sizePolicy);
        label_79->setMinimumSize(QSize(50, 20));
        label_79->setMaximumSize(QSize(50, 20));
        label_79->setFont(font2);
        label_79->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_79, 7, 0, 3, 1);

        LE_JOINT_RHAND = new QLineEdit(gridLayoutWidget);
        LE_JOINT_RHAND->setObjectName(QStringLiteral("LE_JOINT_RHAND"));
        LE_JOINT_RHAND->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_RHAND->sizePolicy().hasHeightForWidth());
        LE_JOINT_RHAND->setSizePolicy(sizePolicy);
        LE_JOINT_RHAND->setMinimumSize(QSize(70, 20));
        LE_JOINT_RHAND->setMaximumSize(QSize(70, 20));
        LE_JOINT_RHAND->setFont(font2);
        LE_JOINT_RHAND->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RHAND->setReadOnly(true);

        gridLayout->addWidget(LE_JOINT_RHAND, 7, 1, 3, 1);

        horizontalLayoutWidget_2 = new QWidget(groupBox);
        horizontalLayoutWidget_2->setObjectName(QStringLiteral("horizontalLayoutWidget_2"));
        horizontalLayoutWidget_2->setGeometry(QRect(10, 30, 281, 31));
        horizontalLayout_2 = new QHBoxLayout(horizontalLayoutWidget_2);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        RB_JOINT_REFERENCE = new QRadioButton(horizontalLayoutWidget_2);
        RB_JOINT_REFERENCE->setObjectName(QStringLiteral("RB_JOINT_REFERENCE"));
        QFont font4;
        font4.setPointSize(9);
        RB_JOINT_REFERENCE->setFont(font4);
        RB_JOINT_REFERENCE->setChecked(true);

        horizontalLayout_2->addWidget(RB_JOINT_REFERENCE);

        RB_JOINT_ENCODER = new QRadioButton(horizontalLayoutWidget_2);
        RB_JOINT_ENCODER->setObjectName(QStringLiteral("RB_JOINT_ENCODER"));
        RB_JOINT_ENCODER->setFont(font4);

        horizontalLayout_2->addWidget(RB_JOINT_ENCODER);

        gridLayoutWidget_2 = new QWidget(groupBox);
        gridLayoutWidget_2->setObjectName(QStringLiteral("gridLayoutWidget_2"));
        gridLayoutWidget_2->setGeometry(QRect(160, 80, 128, 211));
        gridLayout_2 = new QGridLayout(gridLayoutWidget_2);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        label_8 = new QLabel(gridLayoutWidget_2);
        label_8->setObjectName(QStringLiteral("label_8"));
        sizePolicy.setHeightForWidth(label_8->sizePolicy().hasHeightForWidth());
        label_8->setSizePolicy(sizePolicy);
        label_8->setMinimumSize(QSize(50, 20));
        label_8->setMaximumSize(QSize(50, 20));
        label_8->setFont(font2);
        label_8->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_8, 0, 0, 1, 1);

        label_9 = new QLabel(gridLayoutWidget_2);
        label_9->setObjectName(QStringLiteral("label_9"));
        sizePolicy.setHeightForWidth(label_9->sizePolicy().hasHeightForWidth());
        label_9->setSizePolicy(sizePolicy);
        label_9->setMinimumSize(QSize(50, 20));
        label_9->setMaximumSize(QSize(50, 20));
        label_9->setFont(font2);
        label_9->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_9, 3, 0, 1, 1);

        LE_JOINT_LSP = new QLineEdit(gridLayoutWidget_2);
        LE_JOINT_LSP->setObjectName(QStringLiteral("LE_JOINT_LSP"));
        LE_JOINT_LSP->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LSP->sizePolicy().hasHeightForWidth());
        LE_JOINT_LSP->setSizePolicy(sizePolicy);
        LE_JOINT_LSP->setMinimumSize(QSize(70, 20));
        LE_JOINT_LSP->setMaximumSize(QSize(70, 20));
        LE_JOINT_LSP->setFont(font2);
        LE_JOINT_LSP->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LSP->setReadOnly(true);

        gridLayout_2->addWidget(LE_JOINT_LSP, 0, 1, 1, 1);

        label_10 = new QLabel(gridLayoutWidget_2);
        label_10->setObjectName(QStringLiteral("label_10"));
        sizePolicy.setHeightForWidth(label_10->sizePolicy().hasHeightForWidth());
        label_10->setSizePolicy(sizePolicy);
        label_10->setMinimumSize(QSize(50, 20));
        label_10->setMaximumSize(QSize(50, 20));
        label_10->setFont(font2);
        label_10->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_10, 5, 0, 1, 1);

        label_11 = new QLabel(gridLayoutWidget_2);
        label_11->setObjectName(QStringLiteral("label_11"));
        sizePolicy.setHeightForWidth(label_11->sizePolicy().hasHeightForWidth());
        label_11->setSizePolicy(sizePolicy);
        label_11->setMinimumSize(QSize(50, 20));
        label_11->setMaximumSize(QSize(50, 20));
        label_11->setFont(font2);
        label_11->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_11, 4, 0, 1, 1);

        LE_JOINT_LSR = new QLineEdit(gridLayoutWidget_2);
        LE_JOINT_LSR->setObjectName(QStringLiteral("LE_JOINT_LSR"));
        LE_JOINT_LSR->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LSR->sizePolicy().hasHeightForWidth());
        LE_JOINT_LSR->setSizePolicy(sizePolicy);
        LE_JOINT_LSR->setMinimumSize(QSize(70, 20));
        LE_JOINT_LSR->setMaximumSize(QSize(70, 20));
        LE_JOINT_LSR->setFont(font2);
        LE_JOINT_LSR->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LSR->setReadOnly(true);

        gridLayout_2->addWidget(LE_JOINT_LSR, 1, 1, 1, 1);

        label_12 = new QLabel(gridLayoutWidget_2);
        label_12->setObjectName(QStringLiteral("label_12"));
        sizePolicy.setHeightForWidth(label_12->sizePolicy().hasHeightForWidth());
        label_12->setSizePolicy(sizePolicy);
        label_12->setMinimumSize(QSize(50, 20));
        label_12->setMaximumSize(QSize(50, 20));
        label_12->setFont(font2);
        label_12->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_12, 2, 0, 1, 1);

        label_13 = new QLabel(gridLayoutWidget_2);
        label_13->setObjectName(QStringLiteral("label_13"));
        sizePolicy.setHeightForWidth(label_13->sizePolicy().hasHeightForWidth());
        label_13->setSizePolicy(sizePolicy);
        label_13->setMinimumSize(QSize(50, 20));
        label_13->setMaximumSize(QSize(50, 20));
        label_13->setFont(font2);
        label_13->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_13, 1, 0, 1, 1);

        LE_JOINT_LSY = new QLineEdit(gridLayoutWidget_2);
        LE_JOINT_LSY->setObjectName(QStringLiteral("LE_JOINT_LSY"));
        LE_JOINT_LSY->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LSY->sizePolicy().hasHeightForWidth());
        LE_JOINT_LSY->setSizePolicy(sizePolicy);
        LE_JOINT_LSY->setMinimumSize(QSize(70, 20));
        LE_JOINT_LSY->setMaximumSize(QSize(70, 20));
        LE_JOINT_LSY->setFont(font2);
        LE_JOINT_LSY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LSY->setReadOnly(true);

        gridLayout_2->addWidget(LE_JOINT_LSY, 2, 1, 1, 1);

        LE_JOINT_LEB = new QLineEdit(gridLayoutWidget_2);
        LE_JOINT_LEB->setObjectName(QStringLiteral("LE_JOINT_LEB"));
        LE_JOINT_LEB->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LEB->sizePolicy().hasHeightForWidth());
        LE_JOINT_LEB->setSizePolicy(sizePolicy);
        LE_JOINT_LEB->setMinimumSize(QSize(70, 20));
        LE_JOINT_LEB->setMaximumSize(QSize(70, 20));
        LE_JOINT_LEB->setFont(font2);
        LE_JOINT_LEB->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LEB->setReadOnly(true);

        gridLayout_2->addWidget(LE_JOINT_LEB, 3, 1, 1, 1);

        LE_JOINT_LWY = new QLineEdit(gridLayoutWidget_2);
        LE_JOINT_LWY->setObjectName(QStringLiteral("LE_JOINT_LWY"));
        LE_JOINT_LWY->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LWY->sizePolicy().hasHeightForWidth());
        LE_JOINT_LWY->setSizePolicy(sizePolicy);
        LE_JOINT_LWY->setMinimumSize(QSize(70, 20));
        LE_JOINT_LWY->setMaximumSize(QSize(70, 20));
        LE_JOINT_LWY->setFont(font2);
        LE_JOINT_LWY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LWY->setReadOnly(true);

        gridLayout_2->addWidget(LE_JOINT_LWY, 4, 1, 1, 1);

        LE_JOINT_LWP = new QLineEdit(gridLayoutWidget_2);
        LE_JOINT_LWP->setObjectName(QStringLiteral("LE_JOINT_LWP"));
        LE_JOINT_LWP->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LWP->sizePolicy().hasHeightForWidth());
        LE_JOINT_LWP->setSizePolicy(sizePolicy);
        LE_JOINT_LWP->setMinimumSize(QSize(70, 20));
        LE_JOINT_LWP->setMaximumSize(QSize(70, 20));
        LE_JOINT_LWP->setFont(font2);
        LE_JOINT_LWP->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LWP->setReadOnly(true);

        gridLayout_2->addWidget(LE_JOINT_LWP, 5, 1, 1, 1);

        LE_JOINT_LHAND = new QLineEdit(gridLayoutWidget_2);
        LE_JOINT_LHAND->setObjectName(QStringLiteral("LE_JOINT_LHAND"));
        LE_JOINT_LHAND->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LHAND->sizePolicy().hasHeightForWidth());
        LE_JOINT_LHAND->setSizePolicy(sizePolicy);
        LE_JOINT_LHAND->setMinimumSize(QSize(70, 20));
        LE_JOINT_LHAND->setMaximumSize(QSize(70, 20));
        LE_JOINT_LHAND->setFont(font2);
        LE_JOINT_LHAND->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LHAND->setReadOnly(true);

        gridLayout_2->addWidget(LE_JOINT_LHAND, 8, 1, 2, 1);

        label_80 = new QLabel(gridLayoutWidget_2);
        label_80->setObjectName(QStringLiteral("label_80"));
        sizePolicy.setHeightForWidth(label_80->sizePolicy().hasHeightForWidth());
        label_80->setSizePolicy(sizePolicy);
        label_80->setMinimumSize(QSize(50, 20));
        label_80->setMaximumSize(QSize(50, 20));
        label_80->setFont(font2);
        label_80->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_80, 8, 0, 2, 1);

        label_83 = new QLabel(gridLayoutWidget_2);
        label_83->setObjectName(QStringLiteral("label_83"));
        sizePolicy.setHeightForWidth(label_83->sizePolicy().hasHeightForWidth());
        label_83->setSizePolicy(sizePolicy);
        label_83->setMinimumSize(QSize(50, 20));
        label_83->setMaximumSize(QSize(50, 20));
        label_83->setFont(font2);
        label_83->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_83, 6, 0, 2, 1);

        LE_JOINT_LWY2 = new QLineEdit(gridLayoutWidget_2);
        LE_JOINT_LWY2->setObjectName(QStringLiteral("LE_JOINT_LWY2"));
        LE_JOINT_LWY2->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LWY2->sizePolicy().hasHeightForWidth());
        LE_JOINT_LWY2->setSizePolicy(sizePolicy);
        LE_JOINT_LWY2->setMinimumSize(QSize(70, 20));
        LE_JOINT_LWY2->setMaximumSize(QSize(70, 20));
        LE_JOINT_LWY2->setFont(font2);
        LE_JOINT_LWY2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LWY2->setReadOnly(true);

        gridLayout_2->addWidget(LE_JOINT_LWY2, 6, 1, 2, 1);

        gridLayoutWidget_4 = new QWidget(groupBox);
        gridLayoutWidget_4->setObjectName(QStringLiteral("gridLayoutWidget_4"));
        gridLayoutWidget_4->setGeometry(QRect(10, 310, 131, 161));
        gridLayout_4 = new QGridLayout(gridLayoutWidget_4);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        gridLayout_4->setContentsMargins(0, 0, 0, 0);
        LE_JOINT_RAR = new QLineEdit(gridLayoutWidget_4);
        LE_JOINT_RAR->setObjectName(QStringLiteral("LE_JOINT_RAR"));
        LE_JOINT_RAR->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_RAR->sizePolicy().hasHeightForWidth());
        LE_JOINT_RAR->setSizePolicy(sizePolicy);
        LE_JOINT_RAR->setMinimumSize(QSize(70, 20));
        LE_JOINT_RAR->setMaximumSize(QSize(70, 20));
        LE_JOINT_RAR->setFont(font2);
        LE_JOINT_RAR->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RAR->setReadOnly(true);

        gridLayout_4->addWidget(LE_JOINT_RAR, 5, 1, 1, 1);

        label_14 = new QLabel(gridLayoutWidget_4);
        label_14->setObjectName(QStringLiteral("label_14"));
        sizePolicy.setHeightForWidth(label_14->sizePolicy().hasHeightForWidth());
        label_14->setSizePolicy(sizePolicy);
        label_14->setMinimumSize(QSize(50, 20));
        label_14->setMaximumSize(QSize(50, 20));
        label_14->setFont(font2);
        label_14->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_14, 4, 0, 1, 1);

        label_19 = new QLabel(gridLayoutWidget_4);
        label_19->setObjectName(QStringLiteral("label_19"));
        sizePolicy.setHeightForWidth(label_19->sizePolicy().hasHeightForWidth());
        label_19->setSizePolicy(sizePolicy);
        label_19->setMinimumSize(QSize(50, 20));
        label_19->setMaximumSize(QSize(50, 20));
        label_19->setFont(font2);
        label_19->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_19, 5, 0, 1, 1);

        label_20 = new QLabel(gridLayoutWidget_4);
        label_20->setObjectName(QStringLiteral("label_20"));
        sizePolicy.setHeightForWidth(label_20->sizePolicy().hasHeightForWidth());
        label_20->setSizePolicy(sizePolicy);
        label_20->setMinimumSize(QSize(50, 20));
        label_20->setMaximumSize(QSize(50, 20));
        label_20->setFont(font2);
        label_20->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_20, 2, 0, 1, 1);

        LE_JOINT_RHY = new QLineEdit(gridLayoutWidget_4);
        LE_JOINT_RHY->setObjectName(QStringLiteral("LE_JOINT_RHY"));
        LE_JOINT_RHY->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_RHY->sizePolicy().hasHeightForWidth());
        LE_JOINT_RHY->setSizePolicy(sizePolicy);
        LE_JOINT_RHY->setMinimumSize(QSize(70, 20));
        LE_JOINT_RHY->setMaximumSize(QSize(70, 20));
        LE_JOINT_RHY->setFont(font2);
        LE_JOINT_RHY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RHY->setReadOnly(true);

        gridLayout_4->addWidget(LE_JOINT_RHY, 0, 1, 1, 1);

        label_21 = new QLabel(gridLayoutWidget_4);
        label_21->setObjectName(QStringLiteral("label_21"));
        sizePolicy.setHeightForWidth(label_21->sizePolicy().hasHeightForWidth());
        label_21->setSizePolicy(sizePolicy);
        label_21->setMinimumSize(QSize(50, 20));
        label_21->setMaximumSize(QSize(50, 20));
        label_21->setFont(font2);
        label_21->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_21, 0, 0, 1, 1);

        label_22 = new QLabel(gridLayoutWidget_4);
        label_22->setObjectName(QStringLiteral("label_22"));
        sizePolicy.setHeightForWidth(label_22->sizePolicy().hasHeightForWidth());
        label_22->setSizePolicy(sizePolicy);
        label_22->setMinimumSize(QSize(50, 20));
        label_22->setMaximumSize(QSize(50, 20));
        label_22->setFont(font2);
        label_22->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_22, 3, 0, 1, 1);

        LE_JOINT_RHP = new QLineEdit(gridLayoutWidget_4);
        LE_JOINT_RHP->setObjectName(QStringLiteral("LE_JOINT_RHP"));
        LE_JOINT_RHP->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_RHP->sizePolicy().hasHeightForWidth());
        LE_JOINT_RHP->setSizePolicy(sizePolicy);
        LE_JOINT_RHP->setMinimumSize(QSize(70, 20));
        LE_JOINT_RHP->setMaximumSize(QSize(70, 20));
        LE_JOINT_RHP->setFont(font2);
        LE_JOINT_RHP->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RHP->setReadOnly(true);

        gridLayout_4->addWidget(LE_JOINT_RHP, 2, 1, 1, 1);

        LE_JOINT_RHR = new QLineEdit(gridLayoutWidget_4);
        LE_JOINT_RHR->setObjectName(QStringLiteral("LE_JOINT_RHR"));
        LE_JOINT_RHR->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_RHR->sizePolicy().hasHeightForWidth());
        LE_JOINT_RHR->setSizePolicy(sizePolicy);
        LE_JOINT_RHR->setMinimumSize(QSize(70, 20));
        LE_JOINT_RHR->setMaximumSize(QSize(70, 20));
        LE_JOINT_RHR->setFont(font2);
        LE_JOINT_RHR->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RHR->setReadOnly(true);

        gridLayout_4->addWidget(LE_JOINT_RHR, 1, 1, 1, 1);

        LE_JOINT_RKN = new QLineEdit(gridLayoutWidget_4);
        LE_JOINT_RKN->setObjectName(QStringLiteral("LE_JOINT_RKN"));
        LE_JOINT_RKN->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_RKN->sizePolicy().hasHeightForWidth());
        LE_JOINT_RKN->setSizePolicy(sizePolicy);
        LE_JOINT_RKN->setMinimumSize(QSize(70, 20));
        LE_JOINT_RKN->setMaximumSize(QSize(70, 20));
        LE_JOINT_RKN->setFont(font2);
        LE_JOINT_RKN->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RKN->setReadOnly(true);

        gridLayout_4->addWidget(LE_JOINT_RKN, 3, 1, 1, 1);

        label_23 = new QLabel(gridLayoutWidget_4);
        label_23->setObjectName(QStringLiteral("label_23"));
        sizePolicy.setHeightForWidth(label_23->sizePolicy().hasHeightForWidth());
        label_23->setSizePolicy(sizePolicy);
        label_23->setMinimumSize(QSize(50, 20));
        label_23->setMaximumSize(QSize(50, 20));
        label_23->setFont(font2);
        label_23->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_23, 1, 0, 1, 1);

        LE_JOINT_RAP = new QLineEdit(gridLayoutWidget_4);
        LE_JOINT_RAP->setObjectName(QStringLiteral("LE_JOINT_RAP"));
        LE_JOINT_RAP->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_RAP->sizePolicy().hasHeightForWidth());
        LE_JOINT_RAP->setSizePolicy(sizePolicy);
        LE_JOINT_RAP->setMinimumSize(QSize(70, 20));
        LE_JOINT_RAP->setMaximumSize(QSize(70, 20));
        LE_JOINT_RAP->setFont(font2);
        LE_JOINT_RAP->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RAP->setReadOnly(true);

        gridLayout_4->addWidget(LE_JOINT_RAP, 4, 1, 1, 1);

        gridLayoutWidget_5 = new QWidget(groupBox);
        gridLayoutWidget_5->setObjectName(QStringLiteral("gridLayoutWidget_5"));
        gridLayoutWidget_5->setGeometry(QRect(160, 310, 128, 161));
        gridLayout_6 = new QGridLayout(gridLayoutWidget_5);
        gridLayout_6->setObjectName(QStringLiteral("gridLayout_6"));
        gridLayout_6->setContentsMargins(0, 0, 0, 0);
        LE_JOINT_LAR = new QLineEdit(gridLayoutWidget_5);
        LE_JOINT_LAR->setObjectName(QStringLiteral("LE_JOINT_LAR"));
        LE_JOINT_LAR->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LAR->sizePolicy().hasHeightForWidth());
        LE_JOINT_LAR->setSizePolicy(sizePolicy);
        LE_JOINT_LAR->setMinimumSize(QSize(70, 20));
        LE_JOINT_LAR->setMaximumSize(QSize(70, 20));
        LE_JOINT_LAR->setFont(font2);
        LE_JOINT_LAR->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LAR->setReadOnly(true);

        gridLayout_6->addWidget(LE_JOINT_LAR, 5, 1, 1, 1);

        label_30 = new QLabel(gridLayoutWidget_5);
        label_30->setObjectName(QStringLiteral("label_30"));
        sizePolicy.setHeightForWidth(label_30->sizePolicy().hasHeightForWidth());
        label_30->setSizePolicy(sizePolicy);
        label_30->setMinimumSize(QSize(50, 20));
        label_30->setMaximumSize(QSize(50, 20));
        label_30->setFont(font2);
        label_30->setAlignment(Qt::AlignCenter);

        gridLayout_6->addWidget(label_30, 4, 0, 1, 1);

        label_31 = new QLabel(gridLayoutWidget_5);
        label_31->setObjectName(QStringLiteral("label_31"));
        sizePolicy.setHeightForWidth(label_31->sizePolicy().hasHeightForWidth());
        label_31->setSizePolicy(sizePolicy);
        label_31->setMinimumSize(QSize(50, 20));
        label_31->setMaximumSize(QSize(50, 20));
        label_31->setFont(font2);
        label_31->setAlignment(Qt::AlignCenter);

        gridLayout_6->addWidget(label_31, 5, 0, 1, 1);

        label_32 = new QLabel(gridLayoutWidget_5);
        label_32->setObjectName(QStringLiteral("label_32"));
        sizePolicy.setHeightForWidth(label_32->sizePolicy().hasHeightForWidth());
        label_32->setSizePolicy(sizePolicy);
        label_32->setMinimumSize(QSize(50, 20));
        label_32->setMaximumSize(QSize(50, 20));
        label_32->setFont(font2);
        label_32->setAlignment(Qt::AlignCenter);

        gridLayout_6->addWidget(label_32, 2, 0, 1, 1);

        LE_JOINT_LHY = new QLineEdit(gridLayoutWidget_5);
        LE_JOINT_LHY->setObjectName(QStringLiteral("LE_JOINT_LHY"));
        LE_JOINT_LHY->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LHY->sizePolicy().hasHeightForWidth());
        LE_JOINT_LHY->setSizePolicy(sizePolicy);
        LE_JOINT_LHY->setMinimumSize(QSize(70, 20));
        LE_JOINT_LHY->setMaximumSize(QSize(70, 20));
        LE_JOINT_LHY->setFont(font2);
        LE_JOINT_LHY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LHY->setReadOnly(true);

        gridLayout_6->addWidget(LE_JOINT_LHY, 0, 1, 1, 1);

        label_33 = new QLabel(gridLayoutWidget_5);
        label_33->setObjectName(QStringLiteral("label_33"));
        sizePolicy.setHeightForWidth(label_33->sizePolicy().hasHeightForWidth());
        label_33->setSizePolicy(sizePolicy);
        label_33->setMinimumSize(QSize(50, 20));
        label_33->setMaximumSize(QSize(50, 20));
        label_33->setFont(font2);
        label_33->setAlignment(Qt::AlignCenter);

        gridLayout_6->addWidget(label_33, 0, 0, 1, 1);

        label_34 = new QLabel(gridLayoutWidget_5);
        label_34->setObjectName(QStringLiteral("label_34"));
        sizePolicy.setHeightForWidth(label_34->sizePolicy().hasHeightForWidth());
        label_34->setSizePolicy(sizePolicy);
        label_34->setMinimumSize(QSize(50, 20));
        label_34->setMaximumSize(QSize(50, 20));
        label_34->setFont(font2);
        label_34->setAlignment(Qt::AlignCenter);

        gridLayout_6->addWidget(label_34, 3, 0, 1, 1);

        LE_JOINT_LHP = new QLineEdit(gridLayoutWidget_5);
        LE_JOINT_LHP->setObjectName(QStringLiteral("LE_JOINT_LHP"));
        LE_JOINT_LHP->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LHP->sizePolicy().hasHeightForWidth());
        LE_JOINT_LHP->setSizePolicy(sizePolicy);
        LE_JOINT_LHP->setMinimumSize(QSize(70, 20));
        LE_JOINT_LHP->setMaximumSize(QSize(70, 20));
        LE_JOINT_LHP->setFont(font2);
        LE_JOINT_LHP->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LHP->setReadOnly(true);

        gridLayout_6->addWidget(LE_JOINT_LHP, 2, 1, 1, 1);

        LE_JOINT_LHR = new QLineEdit(gridLayoutWidget_5);
        LE_JOINT_LHR->setObjectName(QStringLiteral("LE_JOINT_LHR"));
        LE_JOINT_LHR->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LHR->sizePolicy().hasHeightForWidth());
        LE_JOINT_LHR->setSizePolicy(sizePolicy);
        LE_JOINT_LHR->setMinimumSize(QSize(70, 20));
        LE_JOINT_LHR->setMaximumSize(QSize(70, 20));
        LE_JOINT_LHR->setFont(font2);
        LE_JOINT_LHR->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LHR->setReadOnly(true);

        gridLayout_6->addWidget(LE_JOINT_LHR, 1, 1, 1, 1);

        LE_JOINT_LKN = new QLineEdit(gridLayoutWidget_5);
        LE_JOINT_LKN->setObjectName(QStringLiteral("LE_JOINT_LKN"));
        LE_JOINT_LKN->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LKN->sizePolicy().hasHeightForWidth());
        LE_JOINT_LKN->setSizePolicy(sizePolicy);
        LE_JOINT_LKN->setMinimumSize(QSize(70, 20));
        LE_JOINT_LKN->setMaximumSize(QSize(70, 20));
        LE_JOINT_LKN->setFont(font3);
        LE_JOINT_LKN->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LKN->setReadOnly(true);

        gridLayout_6->addWidget(LE_JOINT_LKN, 3, 1, 1, 1);

        label_35 = new QLabel(gridLayoutWidget_5);
        label_35->setObjectName(QStringLiteral("label_35"));
        sizePolicy.setHeightForWidth(label_35->sizePolicy().hasHeightForWidth());
        label_35->setSizePolicy(sizePolicy);
        label_35->setMinimumSize(QSize(50, 20));
        label_35->setMaximumSize(QSize(50, 20));
        label_35->setFont(font2);
        label_35->setAlignment(Qt::AlignCenter);

        gridLayout_6->addWidget(label_35, 1, 0, 1, 1);

        LE_JOINT_LAP = new QLineEdit(gridLayoutWidget_5);
        LE_JOINT_LAP->setObjectName(QStringLiteral("LE_JOINT_LAP"));
        LE_JOINT_LAP->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LAP->sizePolicy().hasHeightForWidth());
        LE_JOINT_LAP->setSizePolicy(sizePolicy);
        LE_JOINT_LAP->setMinimumSize(QSize(70, 20));
        LE_JOINT_LAP->setMaximumSize(QSize(70, 20));
        LE_JOINT_LAP->setFont(font2);
        LE_JOINT_LAP->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LAP->setReadOnly(true);

        gridLayout_6->addWidget(LE_JOINT_LAP, 4, 1, 1, 1);

        gridLayoutWidget_6 = new QWidget(groupBox);
        gridLayoutWidget_6->setObjectName(QStringLiteral("gridLayoutWidget_6"));
        gridLayoutWidget_6->setGeometry(QRect(10, 490, 128, 61));
        gridLayout_5 = new QGridLayout(gridLayoutWidget_6);
        gridLayout_5->setObjectName(QStringLiteral("gridLayout_5"));
        gridLayout_5->setContentsMargins(0, 0, 0, 0);
        label_27 = new QLabel(gridLayoutWidget_6);
        label_27->setObjectName(QStringLiteral("label_27"));
        sizePolicy.setHeightForWidth(label_27->sizePolicy().hasHeightForWidth());
        label_27->setSizePolicy(sizePolicy);
        label_27->setMinimumSize(QSize(50, 20));
        label_27->setMaximumSize(QSize(50, 20));
        label_27->setFont(font2);
        label_27->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_27, 1, 0, 1, 1);

        label_25 = new QLabel(gridLayoutWidget_6);
        label_25->setObjectName(QStringLiteral("label_25"));
        sizePolicy.setHeightForWidth(label_25->sizePolicy().hasHeightForWidth());
        label_25->setSizePolicy(sizePolicy);
        label_25->setMinimumSize(QSize(50, 20));
        label_25->setMaximumSize(QSize(50, 20));
        label_25->setFont(font2);
        label_25->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_25, 0, 0, 1, 1);

        LE_JOINT_LWH = new QLineEdit(gridLayoutWidget_6);
        LE_JOINT_LWH->setObjectName(QStringLiteral("LE_JOINT_LWH"));
        LE_JOINT_LWH->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_LWH->sizePolicy().hasHeightForWidth());
        LE_JOINT_LWH->setSizePolicy(sizePolicy);
        LE_JOINT_LWH->setMinimumSize(QSize(70, 20));
        LE_JOINT_LWH->setMaximumSize(QSize(70, 20));
        LE_JOINT_LWH->setFont(font2);
        LE_JOINT_LWH->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_LWH->setReadOnly(true);

        gridLayout_5->addWidget(LE_JOINT_LWH, 1, 1, 1, 1);

        LE_JOINT_RWH = new QLineEdit(gridLayoutWidget_6);
        LE_JOINT_RWH->setObjectName(QStringLiteral("LE_JOINT_RWH"));
        LE_JOINT_RWH->setEnabled(false);
        sizePolicy.setHeightForWidth(LE_JOINT_RWH->sizePolicy().hasHeightForWidth());
        LE_JOINT_RWH->setSizePolicy(sizePolicy);
        LE_JOINT_RWH->setMinimumSize(QSize(70, 20));
        LE_JOINT_RWH->setMaximumSize(QSize(70, 20));
        LE_JOINT_RWH->setFont(font2);
        LE_JOINT_RWH->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_JOINT_RWH->setReadOnly(true);

        gridLayout_5->addWidget(LE_JOINT_RWH, 0, 1, 1, 1);

        BTN_ENC_ENABLE = new QPushButton(JointDialog);
        BTN_ENC_ENABLE->setObjectName(QStringLiteral("BTN_ENC_ENABLE"));
        BTN_ENC_ENABLE->setGeometry(QRect(140, 10, 81, 31));
        BTN_ENC_ENABLE->setFont(font4);
        BTN_ENC_DISABLE = new QPushButton(JointDialog);
        BTN_ENC_DISABLE->setObjectName(QStringLiteral("BTN_ENC_DISABLE"));
        BTN_ENC_DISABLE->setGeometry(QRect(230, 10, 81, 31));
        BTN_ENC_DISABLE->setFont(font4);
        line = new QFrame(JointDialog);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(10, 30, 118, 3));
        QFont font5;
        font5.setPointSize(8);
        line->setFont(font5);
        line->setLineWidth(2);
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        retranslateUi(JointDialog);

        QMetaObject::connectSlotsByName(JointDialog);
    } // setupUi

    void retranslateUi(QDialog *JointDialog)
    {
        JointDialog->setWindowTitle(QApplication::translate("JointDialog", "Dialog", 0));
        label->setText(QApplication::translate("JointDialog", "Joint", 0));
        groupBox->setTitle(QApplication::translate("JointDialog", "Joint Reference && Encoder", 0));
        label_18->setText(QApplication::translate("JointDialog", "WST", 0));
        label_5->setText(QApplication::translate("JointDialog", "RWY", 0));
        label_6->setText(QApplication::translate("JointDialog", "RWP", 0));
        label_3->setText(QApplication::translate("JointDialog", "RSY", 0));
        label_2->setText(QApplication::translate("JointDialog", "RSP", 0));
        label_4->setText(QApplication::translate("JointDialog", "REB", 0));
        label_7->setText(QApplication::translate("JointDialog", "RSR", 0));
        label_81->setText(QApplication::translate("JointDialog", "RWY2", 0));
        label_79->setText(QApplication::translate("JointDialog", "RHand", 0));
        RB_JOINT_REFERENCE->setText(QApplication::translate("JointDialog", "Reference", 0));
        RB_JOINT_ENCODER->setText(QApplication::translate("JointDialog", "Encoder", 0));
        label_8->setText(QApplication::translate("JointDialog", "LSP", 0));
        label_9->setText(QApplication::translate("JointDialog", "LEB", 0));
        label_10->setText(QApplication::translate("JointDialog", "LWP", 0));
        label_11->setText(QApplication::translate("JointDialog", "LWY", 0));
        label_12->setText(QApplication::translate("JointDialog", "LSY", 0));
        label_13->setText(QApplication::translate("JointDialog", "LSR", 0));
        label_80->setText(QApplication::translate("JointDialog", "LHand", 0));
        label_83->setText(QApplication::translate("JointDialog", "LWY2", 0));
        label_14->setText(QApplication::translate("JointDialog", "RAP", 0));
        label_19->setText(QApplication::translate("JointDialog", "RAR", 0));
        label_20->setText(QApplication::translate("JointDialog", "RHP", 0));
        label_21->setText(QApplication::translate("JointDialog", "RHY", 0));
        label_22->setText(QApplication::translate("JointDialog", "RKN", 0));
        label_23->setText(QApplication::translate("JointDialog", "RHR", 0));
        label_30->setText(QApplication::translate("JointDialog", "LAP", 0));
        label_31->setText(QApplication::translate("JointDialog", "LAR", 0));
        label_32->setText(QApplication::translate("JointDialog", "LHP", 0));
        label_33->setText(QApplication::translate("JointDialog", "LHY", 0));
        label_34->setText(QApplication::translate("JointDialog", "LKN", 0));
        label_35->setText(QApplication::translate("JointDialog", "LHR", 0));
        label_27->setText(QApplication::translate("JointDialog", "LWH", 0));
        label_25->setText(QApplication::translate("JointDialog", "RWH", 0));
        BTN_ENC_ENABLE->setText(QApplication::translate("JointDialog", "Enc.Enable", 0));
        BTN_ENC_DISABLE->setText(QApplication::translate("JointDialog", "Enc. Disable", 0));
    } // retranslateUi

};

namespace Ui {
    class JointDialog: public Ui_JointDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_JOINTDIALOG_H
