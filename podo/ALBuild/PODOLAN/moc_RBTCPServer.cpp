/****************************************************************************
** Meta object code from reading C++ file 'RBTCPServer.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../ALPrograms/PODOLAN/LAN/RBTCPServer.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'RBTCPServer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_RBTCPServer_t {
    QByteArrayData data[7];
    char stringdata0[96];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_RBTCPServer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_RBTCPServer_t qt_meta_stringdata_RBTCPServer = {
    {
QT_MOC_LITERAL(0, 0, 11), // "RBTCPServer"
QT_MOC_LITERAL(1, 12, 17), // "SIG_NewConnection"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 16), // "SIG_DisConnected"
QT_MOC_LITERAL(4, 48, 15), // "RBNewConnection"
QT_MOC_LITERAL(5, 64, 20), // "RBClientDisconnected"
QT_MOC_LITERAL(6, 85, 10) // "RBReadData"

    },
    "RBTCPServer\0SIG_NewConnection\0\0"
    "SIG_DisConnected\0RBNewConnection\0"
    "RBClientDisconnected\0RBReadData"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_RBTCPServer[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   39,    2, 0x06 /* Public */,
       3,    0,   40,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    0,   41,    2, 0x08 /* Private */,
       5,    0,   42,    2, 0x08 /* Private */,
       6,    0,   43,    2, 0x09 /* Protected */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void RBTCPServer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        RBTCPServer *_t = static_cast<RBTCPServer *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->SIG_NewConnection(); break;
        case 1: _t->SIG_DisConnected(); break;
        case 2: _t->RBNewConnection(); break;
        case 3: _t->RBClientDisconnected(); break;
        case 4: _t->RBReadData(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (RBTCPServer::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&RBTCPServer::SIG_NewConnection)) {
                *result = 0;
            }
        }
        {
            typedef void (RBTCPServer::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&RBTCPServer::SIG_DisConnected)) {
                *result = 1;
            }
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject RBTCPServer::staticMetaObject = {
    { &QTcpServer::staticMetaObject, qt_meta_stringdata_RBTCPServer.data,
      qt_meta_data_RBTCPServer,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *RBTCPServer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *RBTCPServer::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_RBTCPServer.stringdata0))
        return static_cast<void*>(const_cast< RBTCPServer*>(this));
    return QTcpServer::qt_metacast(_clname);
}

int RBTCPServer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QTcpServer::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void RBTCPServer::SIG_NewConnection()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void RBTCPServer::SIG_DisConnected()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
