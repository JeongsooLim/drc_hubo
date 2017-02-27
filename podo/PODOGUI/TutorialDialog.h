#ifndef TUTORIALDIALOG_H
#define TUTORIALDIALOG_H

#include <QDialog>
#include "CommonHeader.h"

namespace Ui {
class TutorialDialog;
}

class TutorialDialog : public QDialog
{
    Q_OBJECT

public:
    explicit TutorialDialog(QWidget *parent = 0);
    ~TutorialDialog();

private slots:
    void on_Timer();

    void on_BTN_TEST_clicked();

    void on_BTN_SCAN_clicked();

    void on_BTN_GOTO_clicked();

    void on_BTN_SW_READY_clicked();

    void on_BTN_SW_SET_clicked();

    void on_BTN_SW_GRASP_clicked();

    void on_BTN_SW_HOLD_clicked();

    void on_BTN_SW_BACK_clicked();

    void on_BTN_SW_GET_POS_clicked();

    void on_BTN_ROSMODE_clicked();

    void on_BTN_GUIMODE_clicked();

    void on_BTN_HOME_POS_clicked();

    void on_BTN_WALK_READY_clicked();

    void on_BTN_TO_WHEEL_clicked();

    void on_BTN_TO_WALK_clicked();

    void on_OW_GOTO_DES_clicked();

private:
    Ui::TutorialDialog *ui;
    QTimer  *timer;

    int ALNum_Tuto;
    int ALNum_Demo;
    int ALNum_Wheel;
    int ALNum_WalkReady;
};

#endif // TUTORIALDIALOG_H
