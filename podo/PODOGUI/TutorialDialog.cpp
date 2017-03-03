#include "TutorialDialog.h"
#include "ui_TutorialDialog.h"

#include "BasicFiles/PODOALDialog.h"


enum DebrisALCOMMAND
{
    DemoFireAL_NO_ACT = 100,
    DemoFireAL_READYPOS,
    DemoFireAL_MOTIONSTART,
    DemoFireAL_TEST,
    DemoFireAL_SET,
    DemoFireAL_SW_READY,
    DemoFireAL_SW_SET,
    DemoFireAL_SW_GRASP,
    DemoFireAL_SW_HOLD,
    DemoFireAL_SW_BACK
};
enum OMNIWHEEL_ALCOMMAND
{
    OMNIWHEEL_AL_NO_ACT = 100,
    OMNIWHEEL_AL_GOTODES,
    OMNIWHEEL_AL_VELMODE,
    OMNIWHEEL_AL_CHANGEPOS,
    OMNIWHEEL_AL_CONTROL,
    OMNIWHEEL_AL_MANUAL,
    OMNIWHEEL_AL_RADIUS,
    OMNIWHEEL_AL_ROS
};
enum WALKREADYCOMMAND
{
    WALKREADY_NO_ACT = 100,
    WALKREADY_GO_WALKREADYPOS,
    WALKREADY_GO_HOMEPOS,
    WALKREADY_WHEELCMD
};

TutorialDialog::TutorialDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::TutorialDialog)
{
    ui->setupUi(this);
    ALNum_Tuto = PODOALDialog::GetALNumFromFileName("ALTutorial");
    ALNum_Demo = PODOALDialog::GetALNumFromFileName("DemoFireAL");
    ALNum_Wheel = PODOALDialog::GetALNumFromFileName("OmniWheel");
    ALNum_WalkReady = PODOALDialog::GetALNumFromFileName("WalkReady");

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(on_Timer()));
    timer->start(100);
}

TutorialDialog::~TutorialDialog()
{
    delete ui;
}

void TutorialDialog::on_Timer(){
    ui->LB_SIMTIME->setText(QString().sprintf("Sim Time :\n%.3f", PODO_DATA.CoreSEN.Sim_Time_sec + PODO_DATA.CoreSEN.Sim_Time_nsec/1000000000.0));
}

void TutorialDialog::on_BTN_TEST_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = 999;
    cmd.COMMAND_TARGET = ALNum_Tuto;
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_SCAN_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = NO_ACT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;

    pLAN->G2MData->ros_cmd.param_f[0] = ui->LE_SCAN_START->text().toFloat()*D2R;
    pLAN->G2MData->ros_cmd.param_f[1] = ui->LE_SCAN_END->text().toFloat()*D2R;
    pLAN->G2MData->ros_cmd.param_f[2] = ui->LE_SCAN_TIME->text().toFloat();
    pLAN->G2MData->ros_cmd.cmd = 1;

    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_GOTO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = NO_ACT;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;

    pLAN->G2MData->ros_cmd.param_f[0] = ui->LE_GOTO_ANGLE->text().toFloat()*D2R;
    pLAN->G2MData->ros_cmd.cmd = 2;

    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_SW_READY_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DemoFireAL_SW_READY;
    cmd.COMMAND_TARGET = ALNum_Demo;
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_SW_SET_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DemoFireAL_SW_SET;

    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_SW_PX->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_SW_PY->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->LE_SW_PZ->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->LE_SW_WZ->text().toDouble();

    cmd.COMMAND_TARGET = ALNum_Demo;
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_SW_GRASP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DemoFireAL_SW_GRASP;
    cmd.COMMAND_TARGET = ALNum_Demo;
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_SW_HOLD_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DemoFireAL_SW_HOLD;
    cmd.COMMAND_TARGET = ALNum_Demo;
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_SW_BACK_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DemoFireAL_SW_BACK;
    cmd.COMMAND_TARGET = ALNum_Demo;
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_SW_GET_POS_clicked()
{
    ui->LE_SW_PX->setText(QString().sprintf("%.3f", PODO_DATA.UserM2G.obj_pos[0]));
    ui->LE_SW_PY->setText(QString().sprintf("%.3f", PODO_DATA.UserM2G.obj_pos[1]));
    ui->LE_SW_PZ->setText(QString().sprintf("%.3f", PODO_DATA.UserM2G.obj_pos[2]));
}

void TutorialDialog::on_BTN_ROSMODE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_Wheel;
    cmd.COMMAND_DATA.USER_COMMAND = OMNIWHEEL_AL_ROS;
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_GUIMODE_clicked()
{
    // E Stop
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]=0;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0]=0;
    cmd.COMMAND_DATA.USER_COMMAND = OMNIWHEEL_AL_VELMODE;
    cmd.COMMAND_TARGET = ALNum_Wheel;
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_HOME_POS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_WalkReady;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_HOMEPOS;
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_WALK_READY_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = ALNum_WalkReady;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 3;
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_TO_WHEEL_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]=0;
    cmd.COMMAND_DATA.USER_COMMAND = OMNIWHEEL_AL_CHANGEPOS;
    cmd.COMMAND_TARGET = ALNum_Wheel;
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_BTN_TO_WALK_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0]=1;
    cmd.COMMAND_DATA.USER_COMMAND = OMNIWHEEL_AL_CHANGEPOS;
    cmd.COMMAND_TARGET = ALNum_Wheel;
    pLAN->SendCommand(cmd);
}

void TutorialDialog::on_OW_GOTO_DES_clicked()
{
    // GOTO DESTINATION
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0]=ui->OW_EDIT_X_POS->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1]=ui->OW_EDIT_Y_POS->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2]=ui->OW_EDIT_T_POS->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3]=360.0;
    cmd.COMMAND_DATA.USER_COMMAND = OMNIWHEEL_AL_GOTODES;
    cmd.COMMAND_TARGET = ALNum_Wheel;
    pLAN->SendCommand(cmd);
}
