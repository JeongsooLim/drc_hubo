#ifndef PODOALDIALOG_H
#define PODOALDIALOG_H

#include <QDialog>
#include <QListWidgetItem>

#include "CommonHeader.h"


namespace Ui {
class PODOALDialog;
}

class PODOALDialog : public QDialog
{
    Q_OBJECT

public:
    explicit PODOALDialog(QWidget *parent = 0);
    ~PODOALDialog();

    static int  GetALNumFromALName(QString ALName);
    static int  GetALNumFromFileName(QString FileName);

protected:
    virtual void hideEvent(QHideEvent *){emit SIG_AL_HIDE();}

signals:
    void SIG_AL_HIDE();

private slots:
    void on_BTN_OPEN_PROCESS_clicked();
    void on_BTN_CLOSE_PROCESS_clicked();
    void on_LTW_PROCESS_LIST_itemPressed(QListWidgetItem *item);
    void on_LTW_PROCESS_LIST_itemDoubleClicked(QListWidgetItem *item);

    void UpdateALStatus();

private:
    Ui::PODOALDialog *ui;
};

#endif // PODOALDIALOG_H
