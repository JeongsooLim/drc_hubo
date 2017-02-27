#include "GUIMainWindow.h"
#include "ui_GUIMainWindow.h"

#include <iostream>



// Database --------
DB_GENERAL      RBDataBase::_DB_GENERAL;
DB_MC           RBDataBase::_DB_MC[MAX_MC];
DB_FT           RBDataBase::_DB_FT[MAX_FT];
DB_IMU          RBDataBase::_DB_IMU[MAX_IMU];
DB_SP           RBDataBase::_DB_SP[MAX_SP];
DB_OF           RBDataBase::_DB_OF[MAX_OF];
DB_AL           RBDataBase::_DB_AL[MAX_AL];

// LAN Data --------
LAN_PODO2GUI    PODO_DATA;
LANDialog       *pLAN;


using namespace std;

GUIMainWindow::GUIMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::GUIMainWindow)
{
    ui->setupUi(this);

    RBDataBase DB;
    DB.SetFilename("Core_Config.db");
    if(DB.OpenDB() == false){
        FILE_LOG(logERROR) << "Fail to load database file";
    }


    dialogLAN = new LANDialog(this);
    pLAN = dialogLAN;
    dialogPODOAL = new PODOALDialog(this);
    dialogSETTING = new SettingDialog(this);
    ui->MAIN_TAB->addTab((QWidget*)dialogSETTING, "Setting");

    // Expandable Dialogs=============================
    expHandler  = new ExpandDialogHandler(this);
    frameJOINT  = new QFrame(this);
    frameSENSOR = new QFrame(this);
    frameMODEL  = new QFrame(this);

    dialogJOINT  = new JointDialog(frameJOINT);
    dialogSENSOR = new SensorDialog(frameSENSOR);
    dialogMODEL  = new ModelDialog(frameMODEL);

    expHandler->registerDialog(dialogJOINT, frameJOINT);
    expHandler->registerDialog(dialogSENSOR, frameSENSOR);
    expHandler->registerDialog(dialogMODEL, frameMODEL);
    // ===============================================

    dialogTUTO = new TutorialDialog(this);
    ui->MAIN_TAB->addTab((QWidget*)dialogTUTO, "Tutorial");





    icon_LAN_OFF.addFile(QStringLiteral("../SHARE/GUI/icon/LAN_OFF_BLACK.png"), QSize(), QIcon::Normal, QIcon::Off);
    icon_LAN_OFF.addFile(QStringLiteral("../SHARE/GUI/icon/LAN_OFF_BLUE.png"), QSize(), QIcon::Normal, QIcon::On);
    icon_LAN_ON.addFile(QStringLiteral("../SHARE/GUI/icon/LAN_ON_BLACK.png"), QSize(), QIcon::Normal, QIcon::Off);
    icon_LAN_ON.addFile(QStringLiteral("../SHARE/GUI/icon/LAN_ON_BLUE.png"), QSize(), QIcon::Normal, QIcon::On);

    // Toolbar Signals================================
    connect(ui->actionLAN, SIGNAL(toggled(bool)), this, SLOT(ActionLAN_Toggled(bool)));
    connect(ui->actionPODOAL, SIGNAL(toggled(bool)), this, SLOT(ActionPODOAL_Toggled(bool)));

    connect(ui->actionJOINT,  SIGNAL(toggled(bool)), this, SLOT(ActionJOINT_Toggled()));
    connect(ui->actionSENSOR, SIGNAL(toggled(bool)), this, SLOT(ActionSENSOR_Toggled()));
    connect(ui->actionMODEL,  SIGNAL(toggled(bool)), this, SLOT(ActionMODEL_Toggled()));
    // ===============================================
    connect(dialogLAN, SIGNAL(SIG_LAN_ON_OFF(bool)), this, SLOT(SLOT_LAN_ONOFF(bool)));
    connect(dialogLAN, SIGNAL(SIG_LAN_HIDE()), this, SLOT(SLOT_LAN_HIDE()));
    connect(dialogPODOAL, SIGNAL(SIG_AL_HIDE()), this, SLOT(SLOT_AL_HIDE()));
}

GUIMainWindow::~GUIMainWindow()
{
    delete ui;
}

void GUIMainWindow::ActionLAN_Toggled(bool checked){
    if(checked){
        dialogLAN->setWindowFlags(Qt::Popup);
        QPoint pt(10,10);
        pt = ui->centralWidget->mapToGlobal(pt);
        dialogLAN->move(pt);
        dialogLAN->show();
    }
}
void GUIMainWindow::ActionPODOAL_Toggled(bool checked){
    if(checked){
        dialogPODOAL->setWindowFlags(Qt::Popup);
        QPoint pt(10,100);
        pt = ui->centralWidget->mapToGlobal(pt);
        dialogPODOAL->move(pt);
        dialogPODOAL->show();
    }
}
void GUIMainWindow::SLOT_LAN_HIDE() {ui->actionLAN->setChecked(false);}
void GUIMainWindow::SLOT_AL_HIDE()  {ui->actionPODOAL->setChecked(false);}
void GUIMainWindow::SLOT_LAN_ONOFF(bool onoff){
    if(onoff){
        ui->actionLAN->setIcon(icon_LAN_ON);
    }else{
        ui->actionLAN->setIcon(icon_LAN_OFF);
    }
}





void GUIMainWindow::ActionJOINT_Toggled(){
    if(expHandler->isVisible(dialogJOINT))
        expHandler->hideDialog(dialogJOINT);
    else
        expHandler->showDialog(dialogJOINT);
}
void GUIMainWindow::ActionSENSOR_Toggled(){
    if(expHandler->isVisible(dialogSENSOR))
        expHandler->hideDialog(dialogSENSOR);
    else
        expHandler->showDialog(dialogSENSOR);
}
void GUIMainWindow::ActionMODEL_Toggled(){
    if(expHandler->isVisible(dialogMODEL))
        expHandler->hideDialog(dialogMODEL);
    else
        expHandler->showDialog(dialogMODEL);
}
