#include "SettingDialog.h"
#include "ui_SettingDialog.h"

enum _SET_JointSequentialNumber_0{
    S_RHY = 0, S_RHR, S_RHP, S_RKN, S_RAP, S_RAR, S_RWH, S_NUM_0
};
enum _SET_JointSequentialNumber_1{
    S_LHY = 0, S_LHR, S_LHP, S_LKN, S_LAP, S_LAR, S_LWH, S_WST, S_NUM_1
};
enum _SET_JointSequentialNumber_2{
    S_RSP = 0, S_RSR, S_RSY, S_REB, S_RWY, S_RWP, S_RWY2, S_RHAND, S_NUM_2
};
enum _SET_JointSequentialNumber_3{
    S_LSP = 0, S_LSR, S_LSY, S_LEB, S_LWY, S_LWP, S_LWY2, S_LHAND, S_NUM_3
};


QString table_0_joint_name[S_NUM_0] = {
    "RHY", "RHR", "RHP", "RKN", "RAP", "RAR", "RWH"
};
QString table_1_joint_name[S_NUM_1] = {
    "LHY", "LHR", "LHP", "LKN", "LAP", "LAR", "LWH", "WST"
};
QString table_2_joint_name[S_NUM_2] = {
    "RSP", "RSR", "RSY", "REB", "RWY", "RWP", "RWY2", "RHand"
};
QString table_3_joint_name[S_NUM_3] = {
    "LSP", "LSR", "LSY", "LEB", "LWY", "LWP", "LWY2", "LHand"
};




const struct {
    int tw;
    int row;
} TW_ROW_Pairs[NO_OF_JOINTS] = {
    {0, 0}, {0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5},
    {1, 0}, {1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5},
    {2, 0}, {2, 1}, {2, 2}, {2, 3}, {2, 4}, {2, 5},
    {3, 0}, {3, 1}, {3, 2}, {3, 3}, {3, 4}, {3, 5},
    {1, 7},
    {2, 6}, {2, 7}, {3, 6}, {3, 7},
    {0, 6}, {1, 6}
};




SettingDialog::SettingDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SettingDialog)
{
    ui->setupUi(this);


    InitTable(ui->TW_0, table_0_joint_name, S_NUM_0);
    InitTable(ui->TW_1, table_1_joint_name, S_NUM_1);
    InitTable(ui->TW_2, table_2_joint_name, S_NUM_2);
    InitTable(ui->TW_3, table_3_joint_name, S_NUM_3);

    lastSelected = RHY;
    select_working = false;
    ChangeSelectedJoint();

    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(UpdateSettings()));
}

SettingDialog::~SettingDialog()
{
    delete ui;
}

void SettingDialog::UpdateSettings(){
    int mcId, mcCh, row;
    QTableWidget *tw;
    QString str;

    for(int i=0; i<NO_OF_JOINTS; i++){
        mcId = MC_GetID(i);
        mcCh = MC_GetCH(i);
        row = TW_ROW_Pairs[i].row;

        switch(TW_ROW_Pairs[i].tw){
        case 0:
            tw = ui->TW_0;
            break;
        case 1:
            tw = ui->TW_1;
            break;
        case 2:
            tw = ui->TW_2;
            break;
        case 3:
            tw = ui->TW_3;
            break;
        }

        mSTAT stat = PODO_DATA.CoreSEN.MCStatus[mcId][mcCh];
        str = "";
        if(stat.b.HIP == 1) str += "H/";
        else                str += "-/";
        if(stat.b.RUN == 1) str += "R/";
        else                str += "-/";
        str += QString().sprintf("%d", stat.b.HME);
        tw->item(row, 0)->setText(str);

        str = "";
        if(stat.b.JAM == 1) str += "JAM ";
        if(stat.b.PWM == 1) str += "PWM ";
        if(stat.b.BIG == 1) str += "BIG ";
        if(stat.b.INP == 1) str += "INP ";
        if(stat.b.FLT == 1) str += "FLT ";
        if(stat.b.ENC == 1) str += "ENC ";
        if(stat.b.CUR == 1) str += "CUR ";
        if(stat.b.TMP == 1) str += "TMP ";
        if(stat.b.PS1 == 1) str += "PS1 ";
        if(stat.b.PS2 == 1) str += "PS2 ";
        if(str == "") str = "-";
        tw->item(row,1)->setText(str);
    }

}

void SettingDialog::InitTable(QTableWidget *table, QString j_names[], int num){
    QFont tableFont;
    tableFont.setPointSize(8);

    const int item_height = 30;
    const int item_width = 50;
    const int col_0_width = 70;
    const int col_1_width = 100;


    // Horizontal - Column
    for(int i=0; i<2; i++){
        table->insertColumn(i);
        table->setHorizontalHeaderItem(i, new QTableWidgetItem());
        table->horizontalHeaderItem(i)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
        table->horizontalHeaderItem(i)->setFont(tableFont);
    }
    table->horizontalHeaderItem(0)->setSizeHint(QSize(col_0_width, item_height));
    table->horizontalHeaderItem(1)->setSizeHint(QSize(col_1_width, item_height));
    table->setColumnWidth(0, col_0_width);
    table->setColumnWidth(1, col_1_width);
    table->horizontalHeaderItem(0)->setText("Status");
    table->horizontalHeaderItem(1)->setText("Error");

    // Vertical - Row
    for(int i=0; i<num; i++){
        table->insertRow(i);
        table->setRowHeight(i,30);
        table->setVerticalHeaderItem(i, new QTableWidgetItem());
        table->verticalHeaderItem(i)->setText(j_names[i]);
        table->verticalHeaderItem(i)->setSizeHint(QSize(item_width, item_height));
        table->verticalHeaderItem(i)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
        table->verticalHeaderItem(i)->setFont(tableFont);
    }

    for(int i=0; i<num; i++){
        for(int j=0; j<2; j++){
            table->setItem(i, j, new QTableWidgetItem());
            table->item(i,j)->setTextAlignment(Qt::AlignHCenter|Qt::AlignVCenter|Qt::AlignCenter);
            table->item(i,j)->setFlags(table->item(i,j)->flags() & ~Qt::ItemIsEditable);
            table->item(i,j)->setFont(tableFont);
        }
    }

    table->setMinimumWidth(item_width + col_0_width + col_1_width + 2);
    table->setMaximumWidth(item_width + col_0_width + col_1_width + 2);
    table->setMinimumHeight(30*(num+1) + 2);
    table->setMaximumHeight(30*(num+1) + 2);
    //table->resizeColumnsToContents();

    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setSelectionMode(QAbstractItemView::SingleSelection);
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    table->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
}

void SettingDialog::UnselectOtherTable(int table){
    QTableWidget *tb;

    for(int i=0; i<4; i++){
        if(i == table)
            continue;

        switch(i){
        case 0:
            tb = ui->TW_0;
            break;
        case 1:
            tb = ui->TW_1;
            break;
        case 2:
            tb = ui->TW_2;
            break;
        case 3:
            tb = ui->TW_3;
            break;
        }

        QList<QTableWidgetItem*> itemlist = tb->selectedItems();
        for(int j=0; j<itemlist.size(); j++){
            itemlist[j]->setSelected(false);
        }
    }
}

void SettingDialog::on_TW_0_itemSelectionChanged(){
    if(select_working == true)
        return;

    select_working = true;
    lastSelected = FindLastSelected(0, ui->TW_0->currentRow());
    UnselectOtherTable(0);
    ChangeSelectedJoint();
    select_working = false;
}
void SettingDialog::on_TW_1_itemSelectionChanged(){
    if(select_working == true)
        return;

    select_working = true;
    lastSelected = FindLastSelected(1, ui->TW_1->currentRow());
    UnselectOtherTable(1);
    ChangeSelectedJoint();
    select_working = false;
}
void SettingDialog::on_TW_2_itemSelectionChanged(){
    if(select_working == true)
        return;

    select_working = true;
    lastSelected = FindLastSelected(2, ui->TW_2->currentRow());
    UnselectOtherTable(2);
    ChangeSelectedJoint();
    select_working = false;
}
void SettingDialog::on_TW_3_itemSelectionChanged(){
    if(select_working == true)
        return;

    select_working = true;
    lastSelected = FindLastSelected(3, ui->TW_3->currentRow());
    UnselectOtherTable(3);
    ChangeSelectedJoint();
    select_working = false;
}

int SettingDialog::FindLastSelected(int tw, int row){
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(TW_ROW_Pairs[i].tw == tw && TW_ROW_Pairs[i].row == row){
            return i;
        }
    }
    return -1;
}
void SettingDialog::ChangeSelectedJoint(){
    ui->LB_SELECTED->setText("Selected: " + JointNameList[lastSelected]);
}


void SettingDialog::on_BTN_CAN_CHECK_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_CHECK_DEVICE;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SettingDialog::on_BTN_FIND_HOME_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_FIND_HOME;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1; // all
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SettingDialog::on_BTN_MOVE_JOINT_clicked(){
    if(lastSelected < 0)
        return;

    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);
    float time = ui->LE_MOVE_TIME->text().toFloat();
    float angle = ui->LE_MOVE_DEGREE->text().toFloat();
    if(lastSelected == RWH || lastSelected == LWH)
        angle /= 200.0;

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;          // relative
    cmd.COMMAND_DATA.USER_PARA_FLOAT[0] = time;      // time(ms)
    cmd.COMMAND_DATA.USER_PARA_FLOAT[1] = angle;	// angle
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_MOTION_MOVE;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SettingDialog::on_BTN_GAIN_OVERRIDE_clicked(){
    if(lastSelected < 0)
        return;

    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);
    float time = ui->LE_GO_TIME->text().toFloat();
    float gain = ui->LE_GO_GAIN->text().toFloat();

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
    cmd.COMMAND_DATA.USER_PARA_FLOAT[0] = time;
    cmd.COMMAND_DATA.USER_PARA_FLOAT[1] = gain;
    cmd.COMMAND_DATA.USER_COMMAND = MOTION_GAIN_OVERRIDE;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}


void SettingDialog::on_BTN_EXECUTE_COMMAND_clicked(){
    if(lastSelected < 0)
        return;

    int id = MC_GetID(lastSelected);
    int ch = MC_GetCH(lastSelected);

    USER_COMMAND cmd;
    if(ui->RB_INIT_POS->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_FIND_HOME;
    }else if(ui->RB_ENC_ZERO->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_ENCODER_RESET;
    }else if(ui->RB_FET_ON->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     //on
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_FET_ONOFF;
    }else if(ui->RB_FET_OFF->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     //off
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_FET_ONOFF;
    }else if(ui->RB_CTRL_ON->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     //on
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF;
    }else if(ui->RB_CTRL_OFF->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     //off
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF;
    }else if(ui->RB_SW_COMP->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;        
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     //complementary
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_ATTR_SWITCHING_MODE;
    }else if(ui->RB_SW_NON_COMP->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     //non-complementary
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_ATTR_SWITCHING_MODE;
    }else if(ui->RB_FRIC_ON->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     //on
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_ATTR_FRICTION_COMPENSATION;
    }else if(ui->RB_FRIC_OFF->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     //off
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_ATTR_FRICTION_COMPENSATION;
    }else if(ui->RB_MODE_POS->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     //position
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_ATTR_CONTROL_MODE;
    }else if(ui->RB_MODE_PWM->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 3;     //pwm
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_ATTR_CONTROL_MODE;
    }else if(ui->RB_ERROR_CLEAR->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;        
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 0;     // only error clear
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_MOTION_ERROR_CLEAR;
    }else if(ui->RB_JOINT_RECOVER->isChecked()){
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = id;
        cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ch;
        cmd.COMMAND_DATA.USER_PARA_CHAR[2] = 1;     // error clear + joint recovery
        cmd.COMMAND_DATA.USER_COMMAND = DAEMON_MOTION_ERROR_CLEAR;
    }
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}
