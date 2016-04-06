#include "PODOALDialog.h"
#include "ui_PODOALDialog.h"

#define AL_ICON_DISABLE_FILE	"../SHARE/GUI/icon/disable.png"
#define AL_ICON_ENABLE_FILE     "../SHARE/GUI/icon/enable.png"


PODOALDialog::PODOALDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PODOALDialog)
{
    ui->setupUi(this);

    FILE_LOG(logINFO) << "ALNUM: " << RBDataBase::_DB_GENERAL.NO_OF_AL;
    for(int i=0; i<RBDataBase::_DB_GENERAL.NO_OF_AL; i++){
        QListWidgetItem *newItem = new QListWidgetItem(QIcon(AL_ICON_DISABLE_FILE), RBDataBase::_DB_AL[i].FileName);
        ui->LTW_PROCESS_LIST->addItem(newItem);
        FILE_LOG(logINFO) << RBDataBase::_DB_AL[i].FileName.toStdString().data();
    }

    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(UpdateALStatus()));
}

PODOALDialog::~PODOALDialog()
{
    delete ui;
}

int PODOALDialog::GetALNumFromALName(QString ALName){
    for(int i=0; i<RBDataBase::_DB_GENERAL.NO_OF_AL; i++){
        if(ALName == RBDataBase::_DB_AL[i].ALName){
            return i;
        }
    }
    return -1;
}
int PODOALDialog::GetALNumFromFileName(QString FileName){
    for(int i=0; i<RBDataBase::_DB_GENERAL.NO_OF_AL; i++){
        if(FileName == RBDataBase::_DB_AL[i].FileName){
            return i;
        }
    }
    return -1;
}


void PODOALDialog::UpdateALStatus(){
    for(int i=0; i<RBDataBase::_DB_GENERAL.NO_OF_AL; i++){
        if(PODO_DATA.CoreSEN.PODO_AL_WORKING[i] == true){
            ui->LTW_PROCESS_LIST->item(i)->setIcon(QIcon(AL_ICON_ENABLE_FILE));
        }else{
            ui->LTW_PROCESS_LIST->item(i)->setIcon(QIcon(AL_ICON_DISABLE_FILE));
        }
    }
}

void PODOALDialog::on_BTN_OPEN_PROCESS_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->LTW_PROCESS_LIST->currentRow();
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_PROCESS_CREATE;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;    
    pLAN->SendCommand(cmd);
}

void PODOALDialog::on_BTN_CLOSE_PROCESS_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->LTW_PROCESS_LIST->currentRow();
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_PROCESS_KILL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;    
    pLAN->SendCommand(cmd);
}

void PODOALDialog::on_LTW_PROCESS_LIST_itemPressed(QListWidgetItem *item){
    if(item->text() == RBDataBase::_DB_AL[0].FileName || item->text() == RBDataBase::_DB_AL[1].FileName){
        ui->BTN_OPEN_PROCESS->setDisabled(true);
        ui->BTN_CLOSE_PROCESS->setDisabled(true);
    }else{
        ui->BTN_OPEN_PROCESS->setEnabled(true);
        ui->BTN_CLOSE_PROCESS->setEnabled(true);
    }
}

void PODOALDialog::on_LTW_PROCESS_LIST_itemDoubleClicked(QListWidgetItem *item){
    int no = 0;
    for(no=0; no<RBDataBase::_DB_GENERAL.NO_OF_AL; no++){
        if(RBDataBase::_DB_AL[no].FileName == item->text()){
            break;
        }
    }
    if(no == RBDataBase::_DB_GENERAL.NO_OF_AL)
        return;
    if(no < 2)
        return;

    if(PODO_DATA.CoreSEN.PODO_AL_WORKING[no] == true){
        on_BTN_CLOSE_PROCESS_clicked();
    }else{
        on_BTN_OPEN_PROCESS_clicked();
    }
}
