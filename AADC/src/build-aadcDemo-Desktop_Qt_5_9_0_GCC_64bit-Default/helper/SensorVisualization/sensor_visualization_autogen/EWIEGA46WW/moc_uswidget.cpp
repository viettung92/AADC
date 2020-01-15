/****************************************************************************
** Meta object code from reading C++ file 'uswidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../aadcDemo/helper/SensorVisualization/uswidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'uswidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_uswidget_t {
    QByteArrayData data[14];
    char stringdata0[165];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_uswidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_uswidget_t qt_meta_stringdata_uswidget = {
    {
QT_MOC_LITERAL(0, 0, 8), // "uswidget"
QT_MOC_LITERAL(1, 9, 7), // "animate"
QT_MOC_LITERAL(2, 17, 0), // ""
QT_MOC_LITERAL(3, 18, 12), // "setDistances"
QT_MOC_LITERAL(4, 31, 12), // "us_side_left"
QT_MOC_LITERAL(5, 44, 13), // "us_side_right"
QT_MOC_LITERAL(6, 58, 12), // "us_rear_left"
QT_MOC_LITERAL(7, 71, 14), // "us_rear_center"
QT_MOC_LITERAL(8, 86, 13), // "us_rear_right"
QT_MOC_LITERAL(9, 100, 9), // "setLSData"
QT_MOC_LITERAL(10, 110, 31), // "aadc::laserscanner::tLaserScan&"
QT_MOC_LITERAL(11, 142, 4), // "scan"
QT_MOC_LITERAL(12, 147, 12), // "setLSMaxDist"
QT_MOC_LITERAL(13, 160, 4) // "dist"

    },
    "uswidget\0animate\0\0setDistances\0"
    "us_side_left\0us_side_right\0us_rear_left\0"
    "us_rear_center\0us_rear_right\0setLSData\0"
    "aadc::laserscanner::tLaserScan&\0scan\0"
    "setLSMaxDist\0dist"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_uswidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   34,    2, 0x0a /* Public */,
       3,    5,   35,    2, 0x0a /* Public */,
       9,    1,   46,    2, 0x0a /* Public */,
      12,    1,   49,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, QMetaType::Int, QMetaType::Int, QMetaType::Int, QMetaType::Int,    4,    5,    6,    7,    8,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void, QMetaType::Double,   13,

       0        // eod
};

void uswidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        uswidget *_t = static_cast<uswidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->animate(); break;
        case 1: _t->setDistances((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4])),(*reinterpret_cast< int(*)>(_a[5]))); break;
        case 2: _t->setLSData((*reinterpret_cast< aadc::laserscanner::tLaserScan(*)>(_a[1]))); break;
        case 3: _t->setLSMaxDist((*reinterpret_cast< double(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject uswidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_uswidget.data,
      qt_meta_data_uswidget,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *uswidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *uswidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_uswidget.stringdata0))
        return static_cast<void*>(const_cast< uswidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int uswidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
