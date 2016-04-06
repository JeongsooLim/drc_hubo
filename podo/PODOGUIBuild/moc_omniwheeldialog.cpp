/****************************************************************************
** Meta object code from reading C++ file 'omniwheeldialog.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../PODOGUI/omniwheeldialog.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'omniwheeldialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_OmniWheelDialog_t {
    QByteArrayData data[17];
    char stringdata0[411];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_OmniWheelDialog_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_OmniWheelDialog_t qt_meta_stringdata_OmniWheelDialog = {
    {
QT_MOC_LITERAL(0, 0, 15), // "OmniWheelDialog"
QT_MOC_LITERAL(1, 16, 19), // "SIG_OMNI_GET_VISION"
QT_MOC_LITERAL(2, 36, 0), // ""
QT_MOC_LITERAL(3, 37, 19), // "SIG_OMNI_GOTO_POINT"
QT_MOC_LITERAL(4, 57, 38), // "SIG_OMNI_SEND_NEW_BOUNDARY_AN..."
QT_MOC_LITERAL(5, 96, 13), // "DisplayUpdate"
QT_MOC_LITERAL(6, 110, 22), // "on_OW_GOTO_DES_clicked"
QT_MOC_LITERAL(7, 133, 18), // "on_OW_STOP_clicked"
QT_MOC_LITERAL(8, 152, 37), // "on_OW_TRANSFORM_WALK_TO_WHEEL..."
QT_MOC_LITERAL(9, 190, 22), // "on_OW_SET_ZERO_clicked"
QT_MOC_LITERAL(10, 213, 30), // "on_OW_NORMAL_WALKREADY_clicked"
QT_MOC_LITERAL(11, 244, 37), // "on_OW_TRANSFORM_WHEEL_TO_WALK..."
QT_MOC_LITERAL(12, 282, 29), // "on_OW_KNEE_GAIN_START_clicked"
QT_MOC_LITERAL(13, 312, 28), // "on_OW_KNEE_GAIN_STOP_clicked"
QT_MOC_LITERAL(14, 341, 22), // "on_SET_TO_REAL_clicked"
QT_MOC_LITERAL(15, 364, 22), // "on_SET_TO_TEST_clicked"
QT_MOC_LITERAL(16, 387, 23) // "on_BTN_ROS_MODE_clicked"

    },
    "OmniWheelDialog\0SIG_OMNI_GET_VISION\0"
    "\0SIG_OMNI_GOTO_POINT\0"
    "SIG_OMNI_SEND_NEW_BOUNDARY_AND_REQUEST\0"
    "DisplayUpdate\0on_OW_GOTO_DES_clicked\0"
    "on_OW_STOP_clicked\0"
    "on_OW_TRANSFORM_WALK_TO_WHEEL_clicked\0"
    "on_OW_SET_ZERO_clicked\0"
    "on_OW_NORMAL_WALKREADY_clicked\0"
    "on_OW_TRANSFORM_WHEEL_TO_WALK_clicked\0"
    "on_OW_KNEE_GAIN_START_clicked\0"
    "on_OW_KNEE_GAIN_STOP_clicked\0"
    "on_SET_TO_REAL_clicked\0on_SET_TO_TEST_clicked\0"
    "on_BTN_ROS_MODE_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_OmniWheelDialog[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   89,    2, 0x06 /* Public */,
       3,    0,   90,    2, 0x06 /* Public */,
       4,    0,   91,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    0,   92,    2, 0x08 /* Private */,
       6,    0,   93,    2, 0x08 /* Private */,
       7,    0,   94,    2, 0x08 /* Private */,
       8,    0,   95,    2, 0x08 /* Private */,
       9,    0,   96,    2, 0x08 /* Private */,
      10,    0,   97,    2, 0x08 /* Private */,
      11,    0,   98,    2, 0x08 /* Private */,
      12,    0,   99,    2, 0x08 /* Private */,
      13,    0,  100,    2, 0x08 /* Private */,
      14,    0,  101,    2, 0x08 /* Private */,
      15,    0,  102,    2, 0x08 /* Private */,
      16,    0,  103,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

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
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void OmniWheelDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        OmniWheelDialog *_t = static_cast<OmniWheelDialog *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->SIG_OMNI_GET_VISION(); break;
        case 1: _t->SIG_OMNI_GOTO_POINT(); break;
        case 2: _t->SIG_OMNI_SEND_NEW_BOUNDARY_AND_REQUEST(); break;
        case 3: _t->DisplayUpdate(); break;
        case 4: _t->on_OW_GOTO_DES_clicked(); break;
        case 5: _t->on_OW_STOP_clicked(); break;
        case 6: _t->on_OW_TRANSFORM_WALK_TO_WHEEL_clicked(); break;
        case 7: _t->on_OW_SET_ZERO_clicked(); break;
        case 8: _t->on_OW_NORMAL_WALKREADY_clicked(); break;
        case 9: _t->on_OW_TRANSFORM_WHEEL_TO_WALK_clicked(); break;
        case 10: _t->on_OW_KNEE_GAIN_START_clicked(); break;
        case 11: _t->on_OW_KNEE_GAIN_STOP_clicked(); break;
        case 12: _t->on_SET_TO_REAL_clicked(); break;
        case 13: _t->on_SET_TO_TEST_clicked(); break;
        case 14: _t->on_BTN_ROS_MODE_clicked(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (OmniWheelDialog::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&OmniWheelDialog::SIG_OMNI_GET_VISION)) {
                *result = 0;
            }
        }
        {
            typedef void (OmniWheelDialog::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&OmniWheelDialog::SIG_OMNI_GOTO_POINT)) {
                *result = 1;
            }
        }
        {
            typedef void (OmniWheelDialog::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&OmniWheelDialog::SIG_OMNI_SEND_NEW_BOUNDARY_AND_REQUEST)) {
                *result = 2;
            }
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject OmniWheelDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_OmniWheelDialog.data,
      qt_meta_data_OmniWheelDialog,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *OmniWheelDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *OmniWheelDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_OmniWheelDialog.stringdata0))
        return static_cast<void*>(const_cast< OmniWheelDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int OmniWheelDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 15)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 15)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 15;
    }
    return _id;
}

// SIGNAL 0
void OmniWheelDialog::SIG_OMNI_GET_VISION()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void OmniWheelDialog::SIG_OMNI_GOTO_POINT()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}

// SIGNAL 2
void OmniWheelDialog::SIG_OMNI_SEND_NEW_BOUNDARY_AND_REQUEST()
{
    QMetaObject::activate(this, &staticMetaObject, 2, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
