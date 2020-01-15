/****************************************************************************
** Meta object code from reading C++ file 'DriverModuleFilter.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../aadcUser/DriverModule/DriverModuleFilter.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'DriverModuleFilter.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_DriverModule_t {
    QByteArrayData data[17];
    char stringdata0[219];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DriverModule_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DriverModule_t qt_meta_stringdata_DriverModule = {
    {
QT_MOC_LITERAL(0, 0, 12), // "DriverModule"
QT_MOC_LITERAL(1, 13, 7), // "SendRun"
QT_MOC_LITERAL(2, 21, 0), // ""
QT_MOC_LITERAL(3, 22, 7), // "entryId"
QT_MOC_LITERAL(4, 30, 8), // "SendStop"
QT_MOC_LITERAL(5, 39, 16), // "SendRequestReady"
QT_MOC_LITERAL(6, 56, 23), // "TriggerLoadManeuverList"
QT_MOC_LITERAL(7, 80, 11), // "OnSendState"
QT_MOC_LITERAL(8, 92, 7), // "tResult"
QT_MOC_LITERAL(9, 100, 20), // "aadc::jury::stateCar"
QT_MOC_LITERAL(10, 121, 7), // "stateID"
QT_MOC_LITERAL(11, 129, 6), // "tInt16"
QT_MOC_LITERAL(12, 136, 16), // "i16ManeuverEntry"
QT_MOC_LITERAL(13, 153, 20), // "TransmitDriverStruct"
QT_MOC_LITERAL(14, 174, 14), // "tDriverStruct&"
QT_MOC_LITERAL(15, 189, 12), // "driverStruct"
QT_MOC_LITERAL(16, 202, 16) // "LoadManeuverList"

    },
    "DriverModule\0SendRun\0\0entryId\0SendStop\0"
    "SendRequestReady\0TriggerLoadManeuverList\0"
    "OnSendState\0tResult\0aadc::jury::stateCar\0"
    "stateID\0tInt16\0i16ManeuverEntry\0"
    "TransmitDriverStruct\0tDriverStruct&\0"
    "driverStruct\0LoadManeuverList"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DriverModule[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   49,    2, 0x06 /* Public */,
       4,    1,   52,    2, 0x06 /* Public */,
       5,    1,   55,    2, 0x06 /* Public */,
       6,    0,   58,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    2,   59,    2, 0x0a /* Public */,
      13,    1,   64,    2, 0x0a /* Public */,
      16,    0,   67,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void,

 // slots: parameters
    0x80000000 | 8, 0x80000000 | 9, 0x80000000 | 11,   10,   12,
    0x80000000 | 8, 0x80000000 | 14,   15,
    0x80000000 | 8,

       0        // eod
};

void DriverModule::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        DriverModule *_t = static_cast<DriverModule *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->SendRun((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->SendStop((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->SendRequestReady((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->TriggerLoadManeuverList(); break;
        case 4: { tResult _r = _t->OnSendState((*reinterpret_cast< aadc::jury::stateCar(*)>(_a[1])),(*reinterpret_cast< tInt16(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< tResult*>(_a[0]) = std::move(_r); }  break;
        case 5: { tResult _r = _t->TransmitDriverStruct((*reinterpret_cast< tDriverStruct(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< tResult*>(_a[0]) = std::move(_r); }  break;
        case 6: { tResult _r = _t->LoadManeuverList();
            if (_a[0]) *reinterpret_cast< tResult*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (DriverModule::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DriverModule::SendRun)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (DriverModule::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DriverModule::SendStop)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (DriverModule::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DriverModule::SendRequestReady)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (DriverModule::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DriverModule::TriggerLoadManeuverList)) {
                *result = 3;
                return;
            }
        }
    }
}

const QMetaObject DriverModule::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_DriverModule.data,
      qt_meta_data_DriverModule,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *DriverModule::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DriverModule::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_DriverModule.stringdata0))
        return static_cast<void*>(const_cast< DriverModule*>(this));
    if (!strcmp(_clname, "cQtUIFilter"))
        return static_cast< cQtUIFilter*>(const_cast< DriverModule*>(this));
    return QObject::qt_metacast(_clname);
}

int DriverModule::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
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
void DriverModule::SendRun(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void DriverModule::SendStop(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void DriverModule::SendRequestReady(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void DriverModule::TriggerLoadManeuverList()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
