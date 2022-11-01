/****************************************************************************
** Meta object code from reading C++ file 'RobotPanel.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "include/RobotPanel.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'RobotPanel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_cpr_rviz__RobotPanel_t {
    QByteArrayData data[11];
    char stringdata0[182];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cpr_rviz__RobotPanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cpr_rviz__RobotPanel_t qt_meta_stringdata_cpr_rviz__RobotPanel = {
    {
QT_MOC_LITERAL(0, 0, 20), // "cpr_rviz::RobotPanel"
QT_MOC_LITERAL(1, 21, 28), // "OnOverrideSliderValueChanged"
QT_MOC_LITERAL(2, 50, 0), // ""
QT_MOC_LITERAL(3, 51, 5), // "value"
QT_MOC_LITERAL(4, 57, 22), // "OnConnectButtonClicked"
QT_MOC_LITERAL(5, 80, 8), // "bChecked"
QT_MOC_LITERAL(6, 89, 21), // "OnOutputButtonClicked"
QT_MOC_LITERAL(7, 111, 3), // "tag"
QT_MOC_LITERAL(8, 115, 21), // "OnEnableButtonClicked"
QT_MOC_LITERAL(9, 137, 24), // "OnReferenceButtonClicked"
QT_MOC_LITERAL(10, 162, 19) // "OnZeroButtonClicked"

    },
    "cpr_rviz::RobotPanel\0OnOverrideSliderValueChanged\0"
    "\0value\0OnConnectButtonClicked\0bChecked\0"
    "OnOutputButtonClicked\0tag\0"
    "OnEnableButtonClicked\0OnReferenceButtonClicked\0"
    "OnZeroButtonClicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cpr_rviz__RobotPanel[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   69,    2, 0x09 /* Protected */,
       4,    1,   72,    2, 0x09 /* Protected */,
       4,    0,   75,    2, 0x29 /* Protected | MethodCloned */,
       6,    2,   76,    2, 0x09 /* Protected */,
       6,    1,   81,    2, 0x29 /* Protected | MethodCloned */,
       8,    1,   84,    2, 0x09 /* Protected */,
       8,    0,   87,    2, 0x29 /* Protected | MethodCloned */,
       9,    1,   88,    2, 0x09 /* Protected */,
       9,    0,   91,    2, 0x29 /* Protected | MethodCloned */,
      10,    1,   92,    2, 0x09 /* Protected */,
      10,    0,   95,    2, 0x29 /* Protected | MethodCloned */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Bool,    5,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, QMetaType::Bool,    7,    5,
    QMetaType::Void, QMetaType::Int,    7,
    QMetaType::Void, QMetaType::Bool,    5,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    5,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    5,
    QMetaType::Void,

       0        // eod
};

void cpr_rviz::RobotPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<RobotPanel *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->OnOverrideSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->OnConnectButtonClicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->OnConnectButtonClicked(); break;
        case 3: _t->OnOutputButtonClicked((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 4: _t->OnOutputButtonClicked((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->OnEnableButtonClicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->OnEnableButtonClicked(); break;
        case 7: _t->OnReferenceButtonClicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->OnReferenceButtonClicked(); break;
        case 9: _t->OnZeroButtonClicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 10: _t->OnZeroButtonClicked(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject cpr_rviz::RobotPanel::staticMetaObject = { {
    &rviz::Panel::staticMetaObject,
    qt_meta_stringdata_cpr_rviz__RobotPanel.data,
    qt_meta_data_cpr_rviz__RobotPanel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *cpr_rviz::RobotPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cpr_rviz::RobotPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_cpr_rviz__RobotPanel.stringdata0))
        return static_cast<void*>(this);
    return rviz::Panel::qt_metacast(_clname);
}

int cpr_rviz::RobotPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz::Panel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 11;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
