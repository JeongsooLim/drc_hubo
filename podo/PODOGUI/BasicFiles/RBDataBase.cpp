#include "RBDataBase.h"

RBDataBase::RBDataBase()
{
    dbCore = QSqlDatabase::addDatabase("QSQLITE");
}

void RBDataBase::SetFilename(QString name){
    filename = name;
}

bool RBDataBase::OpenDB(){
    dbCore.setDatabaseName(filename);
    if(!dbCore.open()){
        FILE_LOG(logERROR) << "Can't open database file [" << filename.toStdString().data() << "]";
        return false;
    }

    QString strQuery;
    QSqlQuery query(dbCore);
    query.setForwardOnly(true);

    // General ----
    query.exec("SELECT * from General");
    if(query.next()){
        _DB_GENERAL.VERSION         = query.value(0).toInt();
        _DB_GENERAL.NO_OF_AL        = query.value(1).toInt();
        _DB_GENERAL.NO_OF_COMM_CH   = query.value(2).toInt();
        _DB_GENERAL.NO_OF_MC        = query.value(3).toInt();
        _DB_GENERAL.NO_OF_FT        = query.value(4).toInt();
        _DB_GENERAL.NO_OF_IMU       = query.value(5).toInt();
        _DB_GENERAL.NO_OF_SP        = query.value(6).toInt();
        _DB_GENERAL.NO_OF_OF        = query.value(7).toInt();
    }

    // PODO AL ----
    strQuery.sprintf("SELECT * from AL");
    query.exec(strQuery);
    for(int i=0; i<_DB_GENERAL.NO_OF_AL; i++){
        if(query.next()){
            _DB_AL[i].ALName          = query.value(0).toString();
            _DB_AL[i].FileName        = query.value(1).toString();
            _DB_AL[i].PathName        = query.value(2).toString();
        }
    }

    // Motor Controller ----
    strQuery.sprintf("SELECT * from MotionController");
    query.exec(strQuery);
    for(int i=0; i<_DB_GENERAL.NO_OF_MC; i++){
        if(query.next()){
            _DB_MC[i].BOARD_ID          = query.value(0).toInt();
            _DB_MC[i].BOARD_TYPE        = query.value(1).toInt();
            _DB_MC[i].MOTOR_CHANNEL     = _DB_MC[i].BOARD_TYPE;
            _DB_MC[i].CAN_CHANNEL       = query.value(2).toInt();
            _DB_MC[i].ID_SEND_REF       = query.value(3).toInt();
            _DB_MC[i].ID_RCV_ENC        = query.value(4).toInt();
            _DB_MC[i].ID_RCV_STAT       = query.value(5).toInt();
            _DB_MC[i].ID_RCV_INFO       = query.value(6).toInt();
            _DB_MC[i].ID_RCV_PARA       = query.value(7).toInt();
            _DB_MC[i].ID_SEND_GENERAL   = query.value(8).toInt();

            for(int j=0; j<MAX_JOINT; j++){
                _DB_MC[i].JOINTS[j].HARMONIC = query.value(9+j*6).toDouble();
                _DB_MC[i].JOINTS[j].PULLY_DRIVE = query.value(10+j*6).toDouble();
                _DB_MC[i].JOINTS[j].PULLY_DRIVEN = query.value(11+j*6).toDouble();
                _DB_MC[i].JOINTS[j].ENCODER_RESOLUTION = query.value(12+j*6).toDouble();
                _DB_MC[i].JOINTS[j].PPR = _DB_MC[i].JOINTS[j].HARMONIC * _DB_MC[i].JOINTS[j].PULLY_DRIVEN / _DB_MC[i].JOINTS[j].PULLY_DRIVE * _DB_MC[i].JOINTS[j].ENCODER_RESOLUTION / 360.0;
                _DB_MC[i].JOINTS[j].FRIC_PARAM1 = query.value(13+j*6).toDouble();
                _DB_MC[i].JOINTS[j].FRIC_PARAM2 = query.value(14+j*6).toDouble();
            }
        }
    }

    // FT Sensor ----
    strQuery.sprintf("SELECT * from FTSensor");
    query.exec(strQuery);
    for(int i=0; i<_DB_GENERAL.NO_OF_FT; i++){
        if(query.next()){
            _DB_FT[i].BOARD_ID      = query.value(0).toInt();
            _DB_FT[i].SENSOR_ID     = query.value(1).toInt();
            _DB_FT[i].CAN_CHANNEL   = query.value(2).toInt();
            _DB_FT[i].SENSOR_TYPE   = query.value(3).toInt();
            _DB_FT[i].ID_RCV_DATA1  = query.value(4).toInt();
            _DB_FT[i].ID_RCV_DATA2  = query.value(5).toInt();
            _DB_FT[i].ID_RCV_ACC    = query.value(6).toInt();
            _DB_FT[i].ID_RCV_STAT   = query.value(7).toInt();
            _DB_FT[i].ID_RCV_INFO   = query.value(8).toInt();
            _DB_FT[i].ID_RCV_PARA   = query.value(9).toInt();
        }
    }

    // IMU Sensor ----
    strQuery.sprintf("SELECT * from IMUSensor");
    query.exec(strQuery);
    for(int i=0; i<_DB_GENERAL.NO_OF_IMU; i++){
        if(query.next()){
            _DB_IMU[i].BOARD_ID     = query.value(0).toInt();
            _DB_IMU[i].SENSOR_ID    = query.value(1).toInt();
            _DB_IMU[i].CAN_CHANNEL  = query.value(2).toInt();
            _DB_IMU[i].SENSOR_TYPE  = query.value(3).toInt();
            _DB_IMU[i].ID_RCV_DATA1 = query.value(4).toInt();
            _DB_IMU[i].ID_RCV_DATA2 = query.value(5).toInt();
            _DB_IMU[i].ID_RCV_STAT  = query.value(6).toInt();
            _DB_IMU[i].ID_RCV_INFO  = query.value(7).toInt();
            _DB_IMU[i].ID_RCV_PARA  = query.value(8).toInt();
        }
    }

    // Smart Power Controller ----
    strQuery.sprintf("SELECT * from SmartPower");
    query.exec(strQuery);
    for(int i=0; i<_DB_GENERAL.NO_OF_SP; i++){
        if(query.next()){
            _DB_SP[i].BOARD_ID      = query.value(0).toInt();
            _DB_SP[i].CAN_CHANNEL   = query.value(1).toInt();
            _DB_SP[i].ID_RCV_DATA   = query.value(2).toInt();
            _DB_SP[i].ID_RCV_INFO   = query.value(4).toInt();
            _DB_SP[i].ID_SEND_GENERAL = query.value(5).toInt();
        }
    }

    // Optic Flow Sensor ----
    strQuery.sprintf("SELECT * from OpticFlowSensor");
    query.exec(strQuery);
    for(int i=0; i<_DB_GENERAL.NO_OF_OF; i++){
        if(query.next()){
            _DB_OF[i].BOARD_ID      = query.value(0).toInt();
            _DB_OF[i].SENSOR_ID     = query.value(1).toInt();
            _DB_OF[i].CAN_CHANNEL   = query.value(2).toInt();
            _DB_OF[i].ID_RCV_DATA   = query.value(3).toInt();
            _DB_OF[i].ID_RCV_INFO   = query.value(4).toInt();
        }
    }

    dbCore.close();
    return true;
}
