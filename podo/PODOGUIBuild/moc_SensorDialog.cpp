/****************************************************************************
** Meta object code from reading C++ file 'SensorDialog.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../PODOGUI/BasicFiles/SensorDialog.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SensorDialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_SensorDialog_t {
    QByteArrayData data[12];
    char stringdata0[295];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SensorDialog_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SensorDialog_t qt_meta_stringdata_SensorDialog = {
    {
QT_MOC_LITERAL(0, 0, 12), // "SensorDialog"
QT_MOC_LITERAL(1, 13, 28), // "on_BTN_SENSOR_ENABLE_clicked"
QT_MOC_LITERAL(2, 42, 0), // ""
QT_MOC_LITERAL(3, 43, 29), // "on_BTN_SENSOR_DISABLE_clicked"
QT_MOC_LITERAL(4, 73, 29), // "on_BTN_SENSOR_FT_NULL_clicked"
QT_MOC_LITERAL(5, 103, 30), // "on_BTN_SENSOR_IMU_NULL_clicked"
QT_MOC_LITERAL(6, 134, 30), // "on_BTN_CIMU_GET_OFFSET_clicked"
QT_MOC_LITERAL(7, 165, 30), // "on_BTN_CIMU_SET_OFFSET_clicked"
QT_MOC_LITERAL(8, 196, 30), // "on_BTN_SENSOR_FOG_ZERO_clicked"
QT_MOC_LITERAL(9, 227, 30), // "on_BTN_SENSOR_FOG_NULL_clicked"
QT_MOC_LITERAL(10, 258, 22), // "on_BTN_OPTZERO_clicked"
QT_MOC_LITERAL(11, 281, 13) // "UpdateSensors"

    },
    "SensorDialog\0on_BTN_SENSOR_ENABLE_clicked\0"
    "\0on_BTN_SENSOR_DISABLE_clicked\0"
    "on_BTN_SENSOR_FT_NULL_clicked\0"
    "on_BTN_SENSOR_IMU_NULL_clicked\0"
    "on_BTN_CIMU_GET_OFFSET_clicked\0"
    "on_BTN_CIMU_SET_OFFSET_clicked\0"
    "on_BTN_SENSOR_FOG_ZERO_clicked\0"
    "on_BTN_SENSOR_FOG_NULL_clicked\0"
    "on_BTN_OPTZERO_clicked\0UpdateSensors"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SensorDialog[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   64,    2, 0x08 /* Private */,
       3,    0,   65,    2, 0x08 /* Private */,
       4,    0,   66,    2, 0x08 /* Private */,
       5,    0,   67,    2, 0x08 /* Private */,
       6,    0,   68,    2, 0x08 /* Private */,
       7,    0,   69,    2, 0x08 /* Private */,
       8,    0,   70,    2, 0x08 /* Private */,
       9,    0,   71,    2, 0x08 /* Private */,
      10,    0,   72,    2, 0x08 /* Private */,
      11,    0,   73,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void SensorDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SensorDialog *_t = static_cast<SensorDialog *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_BTN_SENSOR_ENABLE_clicked(); break;
        case 1: _t->on_BTN_SENSOR_DISABLE_clicked(); break;
        case 2: _t->on_BTN_SENSOR_FT_NULL_clicked(); break;
        case 3: _t->on_BTN_SENSOR_IMU_NULL_clicked(); break;
        case 4: _t->on_BTN_CIMU_GET_OFFSET_clicked(); break;
        case 5: _t->on_BTN_CIMU_SET_OFFSET_clicked(); break;
        case 6: _t->on_BTN_SENSOR_FOG_ZERO_clicked(); break;
        case 7: _t->on_BTN_SENSOR_FOG_NULL_clicked(); break;
        case 8: _t->on_BTN_OPTZERO_clicked(); break;
        case 9: _t->UpdateSensors(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject SensorDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_SensorDialog.data,
      qt_meta_data_SensorDialog,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *SensorDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SensorDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_SensorDialog.stringdata0))
        return static_cast<void*>(const_cast< SensorDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int SensorDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 10;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
