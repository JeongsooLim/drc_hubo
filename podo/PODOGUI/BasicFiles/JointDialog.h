#ifndef JOINTDIALOG_H
#define JOINTDIALOG_H

#include <QDialog>
#include "CommonHeader.h"

namespace Ui {
class JointDialog;
}

class JointDialog : public QDialog
{
    Q_OBJECT

public:
    explicit JointDialog(QWidget *parent = 0);
    ~JointDialog();

private slots:
    void on_BTN_ENC_ENABLE_clicked();
    void on_BTN_ENC_DISABLE_clicked();

    void UpdateJoints();

private:
    Ui::JointDialog *ui;
};

#endif // JOINTDIALOG_H
