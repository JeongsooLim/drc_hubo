/****************************************************************************
** Meta object code from reading C++ file 'LANDialog.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../PODOGUI/BasicFiles/LANDialog.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'LANDialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_LANDialog_t {
    QByteArrayData data[10];
    char stringdata0[130];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_LANDialog_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_LANDialog_t qt_meta_stringdata_LANDialog = {
    {
QT_MOC_LITERAL(0, 0, 9), // "LANDialog"
QT_MOC_LITERAL(1, 10, 11), // "NewPODOData"
QT_MOC_LITERAL(2, 22, 0), // ""
QT_MOC_LITERAL(3, 23, 12), // "SIG_LAN_HIDE"
QT_MOC_LITERAL(4, 36, 14), // "SIG_LAN_ON_OFF"
QT_MOC_LITERAL(5, 51, 5), // "onoff"
QT_MOC_LITERAL(6, 57, 14), // "ReadDataThread"
QT_MOC_LITERAL(7, 72, 26), // "on_BTN_LAN_CONNECT_clicked"
QT_MOC_LITERAL(8, 99, 13), // "LAN_Connected"
QT_MOC_LITERAL(9, 113, 16) // "LAN_Disconnected"

    },
    "LANDialog\0NewPODOData\0\0SIG_LAN_HIDE\0"
    "SIG_LAN_ON_OFF\0onoff\0ReadDataThread\0"
    "on_BTN_LAN_CONNECT_clicked\0LAN_Connected\0"
    "LAN_Disconnected"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_LANDialog[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   49,    2, 0x06 /* Public */,
       3,    0,   50,    2, 0x06 /* Public */,
       4,    1,   51,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    0,   54,    2, 0x08 /* Private */,
       7,    0,   55,    2, 0x08 /* Private */,
       8,    0,   56,    2, 0x08 /* Private */,
       9,    0,   57,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    5,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void LANDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        LANDialog *_t = static_cast<LANDialog *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->NewPODOData(); break;
        case 1: _t->SIG_LAN_HIDE(); break;
        case 2: _t->SIG_LAN_ON_OFF((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->ReadDataThread(); break;
        case 4: _t->on_BTN_LAN_CONNECT_clicked(); break;
        case 5: _t->LAN_Connected(); break;
        case 6: _t->LAN_Disconnected(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (LANDialog::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&LANDialog::NewPODOData)) {
                *result = 0;
            }
        }
        {
            typedef void (LANDialog::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&LANDialog::SIG_LAN_HIDE)) {
                *result = 1;
            }
        }
        {
            typedef void (LANDialog::*_t)(bool );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&LANDialog::SIG_LAN_ON_OFF)) {
                *result = 2;
            }
        }
    }
}

const QMetaObject LANDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_LANDialog.data,
      qt_meta_data_LANDialog,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *LANDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *LANDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_LANDialog.stringdata0))
        return static_cast<void*>(const_cast< LANDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int LANDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void LANDialog::NewPODOData()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void LANDialog::SIG_LAN_HIDE()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}

// SIGNAL 2
void LANDialog::SIG_LAN_ON_OFF(bool _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
struct qt_meta_stringdata_PODOClient_t {
    QByteArrayData data[3];
    char stringdata0[23];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PODOClient_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PODOClient_t qt_meta_stringdata_PODOClient = {
    {
QT_MOC_LITERAL(0, 0, 10), // "PODOClient"
QT_MOC_LITERAL(1, 11, 10), // "RBReadData"
QT_MOC_LITERAL(2, 22, 0) // ""

    },
    "PODOClient\0RBReadData\0"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PODOClient[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   19,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

void PODOClient::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PODOClient *_t = static_cast<PODOClient *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->RBReadData(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject PODOClient::staticMetaObject = {
    { &RBTCPClient::staticMetaObject, qt_meta_stringdata_PODOClient.data,
      qt_meta_data_PODOClient,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *PODOClient::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PODOClient::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_PODOClient.stringdata0))
        return static_cast<void*>(const_cast< PODOClient*>(this));
    return RBTCPClient::qt_metacast(_clname);
}

int PODOClient::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = RBTCPClient::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 1;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
