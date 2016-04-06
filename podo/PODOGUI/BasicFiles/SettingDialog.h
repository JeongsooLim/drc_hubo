#ifndef SETTINGDIALOG_H
#define SETTINGDIALOG_H

#include <QDialog>
#include <QTableWidget>

#include "CommonHeader.h"

namespace Ui {
class SettingDialog;
}

class SettingDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SettingDialog(QWidget *parent = 0);
    ~SettingDialog();

private slots:
    void    UpdateSettings();

    void on_BTN_CAN_CHECK_clicked();
    void on_BTN_FIND_HOME_clicked();
    void on_BTN_MOVE_JOINT_clicked();
    void on_BTN_GAIN_OVERRIDE_clicked();
    void on_BTN_EXECUTE_COMMAND_clicked();

    void on_TW_0_itemSelectionChanged();
    void on_TW_2_itemSelectionChanged();
    void on_TW_1_itemSelectionChanged();
    void on_TW_3_itemSelectionChanged();

private:
    Ui::SettingDialog *ui;

    void    InitTable(QTableWidget *table, QString j_names[], int num);

    int     FindLastSelected(int tw, int row);
    void    ChangeSelectedJoint();
    void    UnselectOtherTable(int table);
    int     select_working;
    int     lastSelected;
};

#endif // SETTINGDIALOG_H
