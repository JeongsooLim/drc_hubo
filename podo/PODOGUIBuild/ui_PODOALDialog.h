/********************************************************************************
** Form generated from reading UI file 'PODOALDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PODOALDIALOG_H
#define UI_PODOALDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_PODOALDialog
{
public:
    QFrame *frame;
    QPushButton *BTN_OPEN_PROCESS;
    QListWidget *LTW_PROCESS_LIST;
    QLabel *label;
    QPushButton *BTN_CLOSE_PROCESS;

    void setupUi(QDialog *PODOALDialog)
    {
        if (PODOALDialog->objectName().isEmpty())
            PODOALDialog->setObjectName(QStringLiteral("PODOALDialog"));
        PODOALDialog->resize(371, 351);
        frame = new QFrame(PODOALDialog);
        frame->setObjectName(QStringLiteral("frame"));
        frame->setGeometry(QRect(0, 0, 371, 351));
        frame->setStyleSheet(QStringLiteral("border-color: rgb(14, 22, 131);"));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Plain);
        frame->setLineWidth(2);
        frame->setMidLineWidth(1);
        BTN_OPEN_PROCESS = new QPushButton(frame);
        BTN_OPEN_PROCESS->setObjectName(QStringLiteral("BTN_OPEN_PROCESS"));
        BTN_OPEN_PROCESS->setGeometry(QRect(235, 92, 121, 51));
        QFont font;
        font.setPointSize(11);
        BTN_OPEN_PROCESS->setFont(font);
        LTW_PROCESS_LIST = new QListWidget(frame);
        LTW_PROCESS_LIST->setObjectName(QStringLiteral("LTW_PROCESS_LIST"));
        LTW_PROCESS_LIST->setGeometry(QRect(15, 72, 190, 260));
        LTW_PROCESS_LIST->setFont(font);
        label = new QLabel(frame);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(25, 22, 281, 31));
        QFont font1;
        font1.setPointSize(17);
        font1.setBold(true);
        font1.setWeight(75);
        label->setFont(font1);
        BTN_CLOSE_PROCESS = new QPushButton(frame);
        BTN_CLOSE_PROCESS->setObjectName(QStringLiteral("BTN_CLOSE_PROCESS"));
        BTN_CLOSE_PROCESS->setGeometry(QRect(235, 212, 121, 51));
        BTN_CLOSE_PROCESS->setFont(font);

        retranslateUi(PODOALDialog);

        QMetaObject::connectSlotsByName(PODOALDialog);
    } // setupUi

    void retranslateUi(QDialog *PODOALDialog)
    {
        PODOALDialog->setWindowTitle(QApplication::translate("PODOALDialog", "Dialog", 0));
        BTN_OPEN_PROCESS->setText(QApplication::translate("PODOALDialog", "Open Process", 0));
        label->setText(QApplication::translate("PODOALDialog", "PODO AL Setting", 0));
        BTN_CLOSE_PROCESS->setText(QApplication::translate("PODOALDialog", "Close Process", 0));
    } // retranslateUi

};

namespace Ui {
    class PODOALDialog: public Ui_PODOALDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PODOALDIALOG_H
