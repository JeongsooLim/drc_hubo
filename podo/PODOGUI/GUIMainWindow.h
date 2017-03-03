#ifndef GUIMAINWINDOW_H
#define GUIMAINWINDOW_H

#include <QMainWindow>
#include "CommonHeader.h"
#include "BasicFiles/LANDialog.h"
#include "BasicFiles/PODOALDialog.h"
#include "BasicFiles/JointDialog.h"
#include "BasicFiles/SensorDialog.h"
#include "BasicFiles/ModelDialog.h"
#include "BasicFiles/SettingDialog.h"


#include "TutorialDialog.h"


namespace Ui {
class GUIMainWindow;
}


class ExpandDialogHandler;


class GUIMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit GUIMainWindow(QWidget *parent = 0);
    ~GUIMainWindow();

private slots:
    void ActionLAN_Toggled(bool checked);
    void ActionPODOAL_Toggled(bool checked);

    void ActionJOINT_Toggled();
    void ActionSENSOR_Toggled();
    void ActionMODEL_Toggled();

    void SLOT_LAN_HIDE();
    void SLOT_AL_HIDE();

    void SLOT_LAN_ONOFF(bool onoff);

private:
    Ui::GUIMainWindow *ui;

    ExpandDialogHandler *expHandler;

    LANDialog       *dialogLAN;
    PODOALDialog    *dialogPODOAL;
    SettingDialog   *dialogSETTING;

    // Expandable Dialog
    JointDialog     *dialogJOINT;
    SensorDialog    *dialogSENSOR;
    ModelDialog     *dialogMODEL;
    QFrame          *frameJOINT;
    QFrame          *frameSENSOR;
    QFrame          *frameMODEL;

    // User Dialog
    TutorialDialog  *dialogTUTO;


    QIcon icon_LAN_ON, icon_LAN_OFF;

};



typedef QVector<QDialog*>	Dialogs;
typedef QVector<QFrame*>	Frames;

class ExpandDialogHandler
{
    friend class PODOMainWindow;
private:
    QWidget		*parent;
    Frames		allFrame;
    Dialogs		allDialog;
    Dialogs		visibleDialog;
    Frames		visibleFrame;

    int			startX;
    int			startY;
    int			height;
    int			sideGap;
public:
    ExpandDialogHandler(QWidget *_parent, int x, int y, int h, int gap)
        : parent(_parent), startX(x), startY(y), height(h), sideGap(gap){
        allFrame.clear();
        allDialog.clear();
        visibleFrame.clear();
        visibleDialog.clear();
    }
    ExpandDialogHandler(QWidget *_parent, int gap = 10, int y = 10)
        : parent(_parent){
        startX = ((GUIMainWindow*)parent)->size().width();
        height = ((GUIMainWindow*)parent)->size().height();
        startY = y;
        sideGap = gap;
        allFrame.clear();
        allDialog.clear();
        visibleFrame.clear();
        visibleDialog.clear();
    }

    void registerDialog(QDialog *dialog, QFrame *frame){
        dialog->setWindowFlags(Qt::Widget);
        frame->setFixedSize(dialog->size());
        dialog->move(0,0);

        allDialog.push_back(dialog);
        allFrame.push_back(frame);

        hideDialog(dialog);
    }
    void showDialog(QDialog *dialog){
        // check already shown
        for(int i=0; i<visibleDialog.size(); i++){
            if(visibleDialog[i] == dialog) return;
        }
        // add show list
        visibleDialog.push_back(dialog);
        for(int i=0; i<allDialog.size(); i++){
            if(dialog == allDialog[i]){
                visibleFrame.push_back(allFrame[i]);
                break;
            }
        }
        refreshDialog();
    }
    void hideDialog(QDialog *dialog){
        for(int i=0; i<visibleDialog.size(); i++){
            if(visibleDialog[i] == dialog){
                visibleDialog.remove(i);
                visibleFrame.remove(i);
                break;
            }
        }
        refreshDialog();
    }
    bool isVisible(QDialog *dialog){
        for(int i=0; i<visibleDialog.size(); i++){
            if(visibleDialog[i] == dialog)
                return true;
        }
        return false;
    }

    void refreshDialog(){
        for(int i=0; i<allDialog.size(); i++){
            allFrame[i]->hide();
        }
        int x = startX;
        for(int i=0; i<visibleDialog.size(); i++){
            visibleFrame[i]->move(x, startY);
            visibleFrame[i]->show();
            x = x + visibleFrame[i]->size().width() + sideGap;
        }
        ((GUIMainWindow*)parent)->setFixedWidth(x);
    }
};


#endif // GUIMAINWINDOW_H

