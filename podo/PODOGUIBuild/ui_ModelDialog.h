/********************************************************************************
** Form generated from reading UI file 'ModelDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MODELDIALOG_H
#define UI_MODELDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ModelDialog
{
public:
    QGroupBox *groupBox_2;
    QCheckBox *CB_USE_ENCODER;
    QCheckBox *CB_SHOW_FLOOR;
    QCheckBox *CB_SHOW_FT;
    QCheckBox *CB_SHOW_WHEEL;
    QWidget *gridLayoutWidget_3;
    QGridLayout *LAYOUT_MODEL;
    QGroupBox *groupBox;
    QPushButton *BT_CamLeft;
    QPushButton *BT_CamFront;
    QPushButton *BT_CamRight;
    QWidget *gridLayoutWidget;
    QGridLayout *LAYOUT_SLIDER;

    void setupUi(QDialog *ModelDialog)
    {
        if (ModelDialog->objectName().isEmpty())
            ModelDialog->setObjectName(QStringLiteral("ModelDialog"));
        ModelDialog->resize(570, 620);
        groupBox_2 = new QGroupBox(ModelDialog);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(150, 530, 261, 81));
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
        CB_USE_ENCODER = new QCheckBox(groupBox_2);
        CB_USE_ENCODER->setObjectName(QStringLiteral("CB_USE_ENCODER"));
        CB_USE_ENCODER->setGeometry(QRect(10, 50, 91, 21));
        CB_USE_ENCODER->setChecked(false);
        CB_SHOW_FLOOR = new QCheckBox(groupBox_2);
        CB_SHOW_FLOOR->setObjectName(QStringLiteral("CB_SHOW_FLOOR"));
        CB_SHOW_FLOOR->setGeometry(QRect(10, 20, 81, 21));
        CB_SHOW_FLOOR->setChecked(false);
        CB_SHOW_FT = new QCheckBox(groupBox_2);
        CB_SHOW_FT->setObjectName(QStringLiteral("CB_SHOW_FT"));
        CB_SHOW_FT->setGeometry(QRect(130, 20, 111, 21));
        CB_SHOW_FT->setChecked(false);
        CB_SHOW_WHEEL = new QCheckBox(groupBox_2);
        CB_SHOW_WHEEL->setObjectName(QStringLiteral("CB_SHOW_WHEEL"));
        CB_SHOW_WHEEL->setGeometry(QRect(130, 50, 121, 21));
        CB_SHOW_WHEEL->setChecked(false);
        gridLayoutWidget_3 = new QWidget(ModelDialog);
        gridLayoutWidget_3->setObjectName(QStringLiteral("gridLayoutWidget_3"));
        gridLayoutWidget_3->setGeometry(QRect(10, 5, 551, 521));
        LAYOUT_MODEL = new QGridLayout(gridLayoutWidget_3);
        LAYOUT_MODEL->setObjectName(QStringLiteral("LAYOUT_MODEL"));
        LAYOUT_MODEL->setContentsMargins(0, 0, 0, 0);
        groupBox = new QGroupBox(ModelDialog);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(10, 530, 131, 81));
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
        BT_CamLeft = new QPushButton(groupBox);
        BT_CamLeft->setObjectName(QStringLiteral("BT_CamLeft"));
        BT_CamLeft->setGeometry(QRect(10, 20, 51, 23));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(BT_CamLeft->sizePolicy().hasHeightForWidth());
        BT_CamLeft->setSizePolicy(sizePolicy);
        BT_CamLeft->setMaximumSize(QSize(500, 16777215));
        BT_CamFront = new QPushButton(groupBox);
        BT_CamFront->setObjectName(QStringLiteral("BT_CamFront"));
        BT_CamFront->setGeometry(QRect(70, 20, 51, 51));
        sizePolicy.setHeightForWidth(BT_CamFront->sizePolicy().hasHeightForWidth());
        BT_CamFront->setSizePolicy(sizePolicy);
        BT_CamFront->setMaximumSize(QSize(500, 16777215));
        BT_CamRight = new QPushButton(groupBox);
        BT_CamRight->setObjectName(QStringLiteral("BT_CamRight"));
        BT_CamRight->setGeometry(QRect(10, 50, 51, 23));
        sizePolicy.setHeightForWidth(BT_CamRight->sizePolicy().hasHeightForWidth());
        BT_CamRight->setSizePolicy(sizePolicy);
        BT_CamRight->setMaximumSize(QSize(500, 16777215));
        gridLayoutWidget = new QWidget(ModelDialog);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(420, 540, 141, 71));
        LAYOUT_SLIDER = new QGridLayout(gridLayoutWidget);
        LAYOUT_SLIDER->setObjectName(QStringLiteral("LAYOUT_SLIDER"));
        LAYOUT_SLIDER->setContentsMargins(0, 0, 0, 0);

        retranslateUi(ModelDialog);

        QMetaObject::connectSlotsByName(ModelDialog);
    } // setupUi

    void retranslateUi(QDialog *ModelDialog)
    {
        ModelDialog->setWindowTitle(QApplication::translate("ModelDialog", "Dialog", 0));
        groupBox_2->setTitle(QApplication::translate("ModelDialog", "Control", 0));
        CB_USE_ENCODER->setText(QApplication::translate("ModelDialog", "Encoder", 0));
        CB_SHOW_FLOOR->setText(QApplication::translate("ModelDialog", "Floor", 0));
        CB_SHOW_FT->setText(QApplication::translate("ModelDialog", "FT Sensor", 0));
        CB_SHOW_WHEEL->setText(QApplication::translate("ModelDialog", "Wheel Speed", 0));
        groupBox->setTitle(QApplication::translate("ModelDialog", "Camera", 0));
        BT_CamLeft->setText(QApplication::translate("ModelDialog", "Left", 0));
        BT_CamFront->setText(QApplication::translate("ModelDialog", "Front", 0));
        BT_CamRight->setText(QApplication::translate("ModelDialog", "Right", 0));
    } // retranslateUi

};

namespace Ui {
    class ModelDialog: public Ui_ModelDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MODELDIALOG_H
