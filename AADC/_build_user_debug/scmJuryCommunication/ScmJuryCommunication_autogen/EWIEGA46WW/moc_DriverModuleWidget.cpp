/****************************************************************************
** Meta object code from reading C++ file 'DriverModuleWidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/aadcUser/scmJuryCommunication/DriverModuleWidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'DriverModuleWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_DisplayWidgetDriver_t {
    QByteArrayData data[18];
    char stringdata0[259];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DisplayWidgetDriver_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DisplayWidgetDriver_t qt_meta_stringdata_DisplayWidgetDriver = {
    {
QT_MOC_LITERAL(0, 0, 19), // "DisplayWidgetDriver"
QT_MOC_LITERAL(1, 20, 10), // "sendStruct"
QT_MOC_LITERAL(2, 31, 0), // ""
QT_MOC_LITERAL(3, 32, 20), // "aadc::jury::stateCar"
QT_MOC_LITERAL(4, 53, 9), // "i8StateID"
QT_MOC_LITERAL(5, 63, 6), // "tInt16"
QT_MOC_LITERAL(6, 70, 16), // "i16ManeuverEntry"
QT_MOC_LITERAL(7, 87, 10), // "OnDriverGo"
QT_MOC_LITERAL(8, 98, 7), // "entryId"
QT_MOC_LITERAL(9, 106, 12), // "OnDriverStop"
QT_MOC_LITERAL(10, 119, 20), // "OnDriverRequestReady"
QT_MOC_LITERAL(11, 140, 16), // "OnStartupClicked"
QT_MOC_LITERAL(12, 157, 17), // "OnStateRunClicked"
QT_MOC_LITERAL(13, 175, 19), // "OnStateErrorClicked"
QT_MOC_LITERAL(14, 195, 22), // "OnResponseReadyClicked"
QT_MOC_LITERAL(15, 218, 22), // "OnStateCompleteClicked"
QT_MOC_LITERAL(16, 241, 12), // "OnAppendText"
QT_MOC_LITERAL(17, 254, 4) // "text"

    },
    "DisplayWidgetDriver\0sendStruct\0\0"
    "aadc::jury::stateCar\0i8StateID\0tInt16\0"
    "i16ManeuverEntry\0OnDriverGo\0entryId\0"
    "OnDriverStop\0OnDriverRequestReady\0"
    "OnStartupClicked\0OnStateRunClicked\0"
    "OnStateErrorClicked\0OnResponseReadyClicked\0"
    "OnStateCompleteClicked\0OnAppendText\0"
    "text"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DisplayWidgetDriver[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   64,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    1,   69,    2, 0x0a /* Public */,
       9,    1,   72,    2, 0x0a /* Public */,
      10,    1,   75,    2, 0x0a /* Public */,
      11,    0,   78,    2, 0x08 /* Private */,
      12,    0,   79,    2, 0x08 /* Private */,
      13,    0,   80,    2, 0x08 /* Private */,
      14,    0,   81,    2, 0x08 /* Private */,
      15,    0,   82,    2, 0x08 /* Private */,
      16,    1,   83,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5,    4,    6,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    8,
    QMetaType::Void, QMetaType::Int,    8,
    QMetaType::Void, QMetaType::Int,    8,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   17,

       0        // eod
};

void DisplayWidgetDriver::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        DisplayWidgetDriver *_t = static_cast<DisplayWidgetDriver *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->sendStruct((*reinterpret_cast< aadc::jury::stateCar(*)>(_a[1])),(*reinterpret_cast< tInt16(*)>(_a[2]))); break;
        case 1: _t->OnDriverGo((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->OnDriverStop((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->OnDriverRequestReady((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->OnStartupClicked(); break;
        case 5: _t->OnStateRunClicked(); break;
        case 6: _t->OnStateErrorClicked(); break;
        case 7: _t->OnResponseReadyClicked(); break;
        case 8: _t->OnStateCompleteClicked(); break;
        case 9: _t->OnAppendText((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (DisplayWidgetDriver::*_t)(aadc::jury::stateCar , tInt16 );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DisplayWidgetDriver::sendStruct)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject DisplayWidgetDriver::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_DisplayWidgetDriver.data,
      qt_meta_data_DisplayWidgetDriver,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *DisplayWidgetDriver::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DisplayWidgetDriver::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_DisplayWidgetDriver.stringdata0))
        return static_cast<void*>(const_cast< DisplayWidgetDriver*>(this));
    return QWidget::qt_metacast(_clname);
}

int DisplayWidgetDriver::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
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

// SIGNAL 0
void DisplayWidgetDriver::sendStruct(aadc::jury::stateCar _t1, tInt16 _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
