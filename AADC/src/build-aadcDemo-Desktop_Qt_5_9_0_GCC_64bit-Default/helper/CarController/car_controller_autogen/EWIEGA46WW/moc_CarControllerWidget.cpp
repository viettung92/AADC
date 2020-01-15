/****************************************************************************
** Meta object code from reading C++ file 'CarControllerWidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../aadcDemo/helper/CarController/CarControllerWidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'CarControllerWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_cCarControllerWidget_t {
    QByteArrayData data[10];
    char stringdata0[111];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cCarControllerWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cCarControllerWidget_t qt_meta_stringdata_cCarControllerWidget = {
    {
QT_MOC_LITERAL(0, 0, 20), // "cCarControllerWidget"
QT_MOC_LITERAL(1, 21, 16), // "steeringReceived"
QT_MOC_LITERAL(2, 38, 0), // ""
QT_MOC_LITERAL(3, 39, 5), // "value"
QT_MOC_LITERAL(4, 45, 16), // "throttleReceived"
QT_MOC_LITERAL(5, 62, 13), // "buttonClicked"
QT_MOC_LITERAL(6, 76, 6), // "button"
QT_MOC_LITERAL(7, 83, 8), // "setSpeed"
QT_MOC_LITERAL(8, 92, 11), // "setSteering"
QT_MOC_LITERAL(9, 104, 6) // "update"

    },
    "cCarControllerWidget\0steeringReceived\0"
    "\0value\0throttleReceived\0buttonClicked\0"
    "button\0setSpeed\0setSteering\0update"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cCarControllerWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   44,    2, 0x06 /* Public */,
       4,    1,   47,    2, 0x06 /* Public */,
       5,    1,   50,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    1,   53,    2, 0x0a /* Public */,
       8,    1,   56,    2, 0x0a /* Public */,
       9,    0,   59,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    6,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void,

       0        // eod
};

void cCarControllerWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        cCarControllerWidget *_t = static_cast<cCarControllerWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->steeringReceived((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->throttleReceived((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->buttonClicked((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->setSpeed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->setSteering((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->update(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (cCarControllerWidget::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&cCarControllerWidget::steeringReceived)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (cCarControllerWidget::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&cCarControllerWidget::throttleReceived)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (cCarControllerWidget::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&cCarControllerWidget::buttonClicked)) {
                *result = 2;
                return;
            }
        }
    }
}

const QMetaObject cCarControllerWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_cCarControllerWidget.data,
      qt_meta_data_cCarControllerWidget,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *cCarControllerWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cCarControllerWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_cCarControllerWidget.stringdata0))
        return static_cast<void*>(const_cast< cCarControllerWidget*>(this));
    if (!strcmp(_clname, "Ui_CarControllerUi"))
        return static_cast< Ui_CarControllerUi*>(const_cast< cCarControllerWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int cCarControllerWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
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
void cCarControllerWidget::steeringReceived(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void cCarControllerWidget::throttleReceived(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void cCarControllerWidget::buttonClicked(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
