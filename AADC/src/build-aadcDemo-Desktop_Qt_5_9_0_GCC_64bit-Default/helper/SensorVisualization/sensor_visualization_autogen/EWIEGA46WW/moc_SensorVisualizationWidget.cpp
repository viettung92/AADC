/****************************************************************************
** Meta object code from reading C++ file 'SensorVisualizationWidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../aadcDemo/helper/SensorVisualization/SensorVisualizationWidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SensorVisualizationWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_cSensorVisualizationWidget_t {
    QByteArrayData data[19];
    char stringdata0[231];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cSensorVisualizationWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cSensorVisualizationWidget_t qt_meta_stringdata_cSensorVisualizationWidget = {
    {
QT_MOC_LITERAL(0, 0, 26), // "cSensorVisualizationWidget"
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
QT_MOC_LITERAL(17, 215, 4), // "scan"
QT_MOC_LITERAL(18, 220, 10) // "update_gui"

    },
    "cSensorVisualizationWidget\0setUs\0\0"
    "tUltrasonicStruct\0usData\0setImu\0"
    "tInerMeasUnitData\0imuData\0setWheelLeft\0"
    "count\0direction\0setWheelRight\0setVoltage\0"
    "tVoltageStruct\0voltData\0setLaserScan\0"
    "aadc::laserscanner::tLaserScan&\0scan\0"
    "update_gui"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cSensorVisualizationWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   49,    2, 0x0a /* Public */,
       5,    1,   52,    2, 0x0a /* Public */,
       8,    2,   55,    2, 0x0a /* Public */,
      11,    2,   60,    2, 0x0a /* Public */,
      12,    1,   65,    2, 0x0a /* Public */,
      15,    1,   68,    2, 0x0a /* Public */,
      18,    0,   71,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,    9,   10,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,    9,   10,
    QMetaType::Void, 0x80000000 | 13,   14,
    QMetaType::Void, 0x80000000 | 16,   17,
    QMetaType::Void,

       0        // eod
};

void cSensorVisualizationWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        cSensorVisualizationWidget *_t = static_cast<cSensorVisualizationWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->setUs((*reinterpret_cast< tUltrasonicStruct(*)>(_a[1]))); break;
        case 1: _t->setImu((*reinterpret_cast< tInerMeasUnitData(*)>(_a[1]))); break;
        case 2: _t->setWheelLeft((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 3: _t->setWheelRight((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 4: _t->setVoltage((*reinterpret_cast< tVoltageStruct(*)>(_a[1]))); break;
        case 5: _t->setLaserScan((*reinterpret_cast< aadc::laserscanner::tLaserScan(*)>(_a[1]))); break;
        case 6: _t->update_gui(); break;
        default: ;
        }
    }
}

const QMetaObject cSensorVisualizationWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_cSensorVisualizationWidget.data,
      qt_meta_data_cSensorVisualizationWidget,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *cSensorVisualizationWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cSensorVisualizationWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_cSensorVisualizationWidget.stringdata0))
        return static_cast<void*>(const_cast< cSensorVisualizationWidget*>(this));
    if (!strcmp(_clname, "Ui_SensorVisualizationUi"))
        return static_cast< Ui_SensorVisualizationUi*>(const_cast< cSensorVisualizationWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int cSensorVisualizationWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
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
QT_WARNING_POP
QT_END_MOC_NAMESPACE
