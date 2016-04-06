/****************************************************************************
** Meta object code from reading C++ file 'PODOALDialog.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../PODOGUI/BasicFiles/PODOALDialog.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'PODOALDialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_PODOALDialog_t {
    QByteArrayData data[10];
    char stringdata0[190];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PODOALDialog_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PODOALDialog_t qt_meta_stringdata_PODOALDialog = {
    {
QT_MOC_LITERAL(0, 0, 12), // "PODOALDialog"
QT_MOC_LITERAL(1, 13, 11), // "SIG_AL_HIDE"
QT_MOC_LITERAL(2, 25, 0), // ""
QT_MOC_LITERAL(3, 26, 27), // "on_BTN_OPEN_PROCESS_clicked"
QT_MOC_LITERAL(4, 54, 28), // "on_BTN_CLOSE_PROCESS_clicked"
QT_MOC_LITERAL(5, 83, 31), // "on_LTW_PROCESS_LIST_itemPressed"
QT_MOC_LITERAL(6, 115, 16), // "QListWidgetItem*"
QT_MOC_LITERAL(7, 132, 4), // "item"
QT_MOC_LITERAL(8, 137, 37), // "on_LTW_PROCESS_LIST_itemDoubl..."
QT_MOC_LITERAL(9, 175, 14) // "UpdateALStatus"

    },
    "PODOALDialog\0SIG_AL_HIDE\0\0"
    "on_BTN_OPEN_PROCESS_clicked\0"
    "on_BTN_CLOSE_PROCESS_clicked\0"
    "on_LTW_PROCESS_LIST_itemPressed\0"
    "QListWidgetItem*\0item\0"
    "on_LTW_PROCESS_LIST_itemDoubleClicked\0"
    "UpdateALStatus"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PODOALDialog[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   44,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    0,   45,    2, 0x08 /* Private */,
       4,    0,   46,    2, 0x08 /* Private */,
       5,    1,   47,    2, 0x08 /* Private */,
       8,    1,   50,    2, 0x08 /* Private */,
       9,    0,   53,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void,

       0        // eod
};

void PODOALDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PODOALDialog *_t = static_cast<PODOALDialog *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->SIG_AL_HIDE(); break;
        case 1: _t->on_BTN_OPEN_PROCESS_clicked(); break;
        case 2: _t->on_BTN_CLOSE_PROCESS_clicked(); break;
        case 3: _t->on_LTW_PROCESS_LIST_itemPressed((*reinterpret_cast< QListWidgetItem*(*)>(_a[1]))); break;
        case 4: _t->on_LTW_PROCESS_LIST_itemDoubleClicked((*reinterpret_cast< QListWidgetItem*(*)>(_a[1]))); break;
        case 5: _t->UpdateALStatus(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (PODOALDialog::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PODOALDialog::SIG_AL_HIDE)) {
                *result = 0;
            }
        }
    }
}

const QMetaObject PODOALDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_PODOALDialog.data,
      qt_meta_data_PODOALDialog,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *PODOALDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PODOALDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_PODOALDialog.stringdata0))
        return static_cast<void*>(const_cast< PODOALDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int PODOALDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void PODOALDialog::SIG_AL_HIDE()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
