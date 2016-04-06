/********************************************************************************
** Form generated from reading UI file 'LANDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LANDIALOG_H
#define UI_LANDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_LANDialog
{
public:
    QFrame *frame;
    QPushButton *BTN_LAN_CONNECT;
    QLabel *label;
    QLineEdit *LE_PORT_NUM;
    QLineEdit *LE_IP_ADDR;

    void setupUi(QDialog *LANDialog)
    {
        if (LANDialog->objectName().isEmpty())
            LANDialog->setObjectName(QStringLiteral("LANDialog"));
        LANDialog->resize(301, 201);
        LANDialog->setAutoFillBackground(false);
        frame = new QFrame(LANDialog);
        frame->setObjectName(QStringLiteral("frame"));
        frame->setGeometry(QRect(0, 0, 301, 201));
        frame->setStyleSheet(QStringLiteral("border-color: rgb(14, 22, 131);"));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Plain);
        frame->setLineWidth(2);
        frame->setMidLineWidth(1);
        BTN_LAN_CONNECT = new QPushButton(frame);
        BTN_LAN_CONNECT->setObjectName(QStringLiteral("BTN_LAN_CONNECT"));
        BTN_LAN_CONNECT->setGeometry(QRect(170, 134, 121, 51));
        QFont font;
        font.setPointSize(15);
        BTN_LAN_CONNECT->setFont(font);
        label = new QLabel(frame);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(20, 20, 181, 31));
        QFont font1;
        font1.setPointSize(17);
        font1.setBold(true);
        font1.setWeight(75);
        label->setFont(font1);
        LE_PORT_NUM = new QLineEdit(frame);
        LE_PORT_NUM->setObjectName(QStringLiteral("LE_PORT_NUM"));
        LE_PORT_NUM->setGeometry(QRect(210, 64, 81, 51));
        LE_PORT_NUM->setFont(font);
        LE_PORT_NUM->setAlignment(Qt::AlignCenter);
        LE_IP_ADDR = new QLineEdit(frame);
        LE_IP_ADDR->setObjectName(QStringLiteral("LE_IP_ADDR"));
        LE_IP_ADDR->setGeometry(QRect(10, 64, 191, 51));
        LE_IP_ADDR->setFont(font);
        LE_IP_ADDR->setAlignment(Qt::AlignCenter);

        retranslateUi(LANDialog);

        QMetaObject::connectSlotsByName(LANDialog);
    } // setupUi

    void retranslateUi(QDialog *LANDialog)
    {
        LANDialog->setWindowTitle(QApplication::translate("LANDialog", "LAN Dialog", 0));
        BTN_LAN_CONNECT->setText(QApplication::translate("LANDialog", "Connect", 0));
        label->setText(QApplication::translate("LANDialog", "LAN Setting", 0));
        LE_PORT_NUM->setText(QApplication::translate("LANDialog", "4000", 0));
        LE_IP_ADDR->setText(QApplication::translate("LANDialog", "127.127.127.127", 0));
    } // retranslateUi

};

namespace Ui {
    class LANDialog: public Ui_LANDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LANDIALOG_H
