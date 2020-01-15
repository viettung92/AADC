/****************************************************************************
** Meta object code from reading C++ file 'SensorVisualizationFilter.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../aadcDemo/helper/SensorVisualization/SensorVisualizationFilter.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SensorVisualizationFilter.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_cSensorVisualizationFilter_t {
    QByteArrayData data[18];
    char stringdata0[220];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cSensorVisualizationFilter_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cSensorVisualizationFilter_t qt_meta_stringdata_cSensorVisualizationFilter = {
    {
QT_MOC_LITERAL(0, 0, 26), // "cSensorVisualizationFilter"
QT_MOC_LITERAL(1, 27, 5), // "setUs"
QT_MOC_LITERAL(2, 33, 0), // ""
QT_MOC_LITERAL(3, 34, 17), // "tUltrasonicStruct"
QT_MOC_LITERAL(4, 52, 6), // "usData"
QT_MOC_LITERAL(5, 59, 6), // "setImu"
QT_MOC_LITERAL(6, 66, 17), // "tInerMeasUnitData"
QT_MOC_LITERAL(7, 84, 7), // "imuData"
QT_MOC_LITERAL(8, 92, 12), // "setWheelLeft"
QT_MOC_LITERAL(9, 105, 5), // "count"
QT_MOC_LITERAL(10, 111, 9), // "direction"
QT_MOC_LITERAL(11, 121, 13), // "setWheelRight"
QT_MOC_LITERAL(12, 135, 10), // "setVoltage"
QT_MOC_LITERAL(13, 146, 14), // "tVoltageStruct"
QT_MOC_LITERAL(14, 161, 8), // "voltData"
QT_MOC_LITERAL(15, 170, 12), // "setLaserScan"
QT_MOC_LITERAL(16, 183, 31), // "aadc::laserscanner::tLaserScan&"
QT_MOC_LITERAL(17, 215, 4) // "scan"

    },
    "cSensorVisualizationFilter\0setUs\0\0"
    "tUltrasonicStruct\0usData\0setImu\0"
    "tInerMeasUnitData\0imuData\0setWheelLeft\0"
    "count\0direction\0setWheelRight\0setVoltage\0"
    "tVoltageStruct\0voltData\0setLaserScan\0"
    "aadc::laserscanner::tLaserScan&\0scan"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cSensorVisualizationFilter[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   44,    2, 0x06 /* Public */,
       5,    1,   47,    2, 0x06 /* Public */,
       8,    2,   50,    2, 0x06 /* Public */,
      11,    2,   55,    2, 0x06 /* Public */,
      12,    1,   60,    2, 0x06 /* Public */,
      15,    1,   63,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,    9,   10,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,    9,   10,
    QMetaType::Void, 0x80000000 | 13,   14,
    QMetaType::Void, 0x80000000 | 16,   17,

       0        // eod
};

void cSensorVisualizationFilter::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        cSensorVisualizationFilter *_t = static_cast<cSensorVisualizationFilter *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->setUs((*reinterpret_cast< tUltrasonicStruct(*)>(_a[1]))); break;
        case 1: _t->setImu((*reinterpret_cast< tInerMeasUnitData(*)>(_a[1]))); break;
        case 2: _t->setWheelLeft((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 3: _t->setWheelRight((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 4: _t->setVoltage((*reinterpret_cast< tVoltageStruct(*)>(_a[1]))); break;
        case 5: _t->setLaserScan((*reinterpret_cast< aadc::laserscanner::tLaserScan(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (cSensorVisualizationFilter::*_t)(tUltrasonicStruct );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&cSensorVisualizationFilter::setUs)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (cSensorVisualizationFilter::*_t)(tInerMeasUnitData );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&cSensorVisualizationFilter::setImu)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (cSensorVisualizationFilter::*_t)(int , int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&cSensorVisualizationFilter::setWheelLeft)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (cSensorVisualizationFilter::*_t)(int , int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&cSensorVisualizationFilter::setWheelRight)) {
                *result = 3;
                return;
            }
        }
        {
            typedef void (cSensorVisualizationFilter::*_t)(tVoltageStruct );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&cSensorVisualizationFilter::setVoltage)) {
                *result = 4;
                return;
            }
        }
        {
            typedef void (cSensorVisualizationFilter::*_t)(aadc::laserscanner::tLaserScan & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&cSensorVisualizationFilter::setLaserScan)) {
                *result = 5;
                return;
            }
        }
    }
}

const QMetaObject cSensorVisualizationFilter::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_cSensorVisualizationFilter.data,
      qt_meta_data_cSensorVisualizationFilter,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *cSensorVisualizationFilter::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cSensorVisualizationFilter::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_cSensorVisualizationFilter.stringdata0))
        return static_cast<void*>(const_cast< cSensorVisualizationFilter*>(this));
    if (!strcmp(_clname, "cQtUIFilter"))
        return static_cast< cQtUIFilter*>(const_cast< cSensorVisualizationFilter*>(this));
    return QObject::qt_metacast(_clname);
}

int cSensorVisualizationFilter::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
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
void cSensorVisualizationFilter::setUs(tUltrasonicStruct _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void cSensorVisualizationFilter::setImu(tInerMeasUnitData _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void cSensorVisualizationFilter::setWheelLeft(int _t1, int _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void cSensorVisualizationFilter::setWheelRight(int _t1, int _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void cSensorVisualizationFilter::setVoltage(tVoltageStruct _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void cSensorVisualizationFilter::setLaserScan(aadc::laserscanner::tLaserScan & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
