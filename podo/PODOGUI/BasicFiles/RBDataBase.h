#ifndef RBDATABASE_H
#define RBDATABASE_H

#include "RBLog.h"
#include "RBDataType.h"

//For Database File
#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlQuery>



class RBDataBase
{
public:
    RBDataBase();

    void    SetFilename(QString name);
    bool    OpenDB();

    static DB_GENERAL      _DB_GENERAL;
    static DB_MC           _DB_MC[MAX_MC];
    static DB_FT           _DB_FT[MAX_FT];
    static DB_IMU          _DB_IMU[MAX_IMU];
    static DB_SP           _DB_SP[MAX_SP];
    static DB_OF           _DB_OF[MAX_OF];
    static DB_AL           _DB_AL[MAX_AL];


private:
    QString         filename;
    QSqlDatabase    dbCore;
};

#endif // RBDATABASE_H
