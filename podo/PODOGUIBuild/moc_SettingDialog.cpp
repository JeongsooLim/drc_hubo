/****************************************************************************
** Meta object code from reading C++ file 'SettingDialog.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../PODOGUI/BasicFiles/SettingDialog.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SettingDialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_SettingDialog_t {
    QByteArrayData data[12];
    char stringdata0[282];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SettingDialog_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SettingDialog_t qt_meta_stringdata_SettingDialog = {
    {
QT_MOC_LITERAL(0, 0, 13), // "SettingDialog"
QT_MOC_LITERAL(1, 14, 14), // "UpdateSettings"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 24), // "on_BTN_CAN_CHECK_clicked"
QT_MOC_LITERAL(4, 55, 24), // "on_BTN_FIND_HOME_clicked"
QT_MOC_LITERAL(5, 80, 25), // "on_BTN_MOVE_JOINT_clicked"
QT_MOC_LITERAL(6, 106, 28), // "on_BTN_GAIN_OVERRIDE_clicked"
QT_MOC_LITERAL(7, 135, 30), // "on_BTN_EXECUTE_COMMAND_clicked"
QT_MOC_LITERAL(8, 166, 28), // "on_TW_0_itemSelectionChanged"
QT_MOC_LITERAL(9, 195, 28), // "on_TW_2_itemSelectionChanged"
QT_MOC_LITERAL(10, 224, 28), // "on_TW_1_itemSelectionChanged"
QT_MOC_LITERAL(11, 253, 28) // "on_TW_3_itemSelectionChanged"

    },
    "SettingDialog\0UpdateSettings\0\0"
    "on_BTN_CAN_CHECK_clicked\0"
    "on_BTN_FIND_HOME_clicked\0"
    "on_BTN_MOVE_JOINT_clicked\0"
    "on_BTN_GAIN_OVERRIDE_clicked\0"
    "on_BTN_EXECUTE_COMMAND_clicked\0"
    "on_TW_0_itemSelectionChanged\0"
    "on_TW_2_itemSelectionChanged\0"
    "on_TW_1_itemSelectionChanged\0"
    "on_TW_3_itemSelectionChanged"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SettingDialog[] = {

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

void SettingDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SettingDialog *_t = static_cast<SettingDialog *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->UpdateSettings(); break;
        case 1: _t->on_BTN_CAN_CHECK_clicked(); break;
        case 2: _t->on_BTN_FIND_HOME_clicked(); break;
        case 3: _t->on_BTN_MOVE_JOINT_clicked(); break;
        case 4: _t->on_BTN_GAIN_OVERRIDE_clicked(); break;
        case 5: _t->on_BTN_EXECUTE_COMMAND_clicked(); break;
        case 6: _t->on_TW_0_itemSelectionChanged(); break;
        case 7: _t->on_TW_2_itemSelectionChanged(); break;
        case 8: _t->on_TW_1_itemSelectionChanged(); break;
        case 9: _t->on_TW_3_itemSelectionChanged(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject SettingDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_SettingDialog.data,
      qt_meta_data_SettingDialog,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *SettingDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SettingDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_SettingDialog.stringdata0))
        return static_cast<void*>(const_cast< SettingDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int SettingDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
