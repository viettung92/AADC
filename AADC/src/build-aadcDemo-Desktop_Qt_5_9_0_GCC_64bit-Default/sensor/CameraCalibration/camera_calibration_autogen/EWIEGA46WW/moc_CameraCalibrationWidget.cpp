/****************************************************************************
** Meta object code from reading C++ file 'CameraCalibrationWidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../aadcDemo/sensor/CameraCalibration/CameraCalibrationWidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'CameraCalibrationWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_cCameraCalibrationWidget_t {
    QByteArrayData data[12];
    char stringdata0[122];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_cCameraCalibrationWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_cCameraCalibrationWidget_t qt_meta_stringdata_cCameraCalibrationWidget = {
    {
QT_MOC_LITERAL(0, 0, 24), // "cCameraCalibrationWidget"
QT_MOC_LITERAL(1, 25, 10), // "SendSaveAs"
QT_MOC_LITERAL(2, 36, 0), // ""
QT_MOC_LITERAL(3, 37, 8), // "filename"
QT_MOC_LITERAL(4, 46, 10), // "OnNewImage"
QT_MOC_LITERAL(5, 57, 8), // "newImage"
QT_MOC_LITERAL(6, 66, 10), // "OnSetState"
QT_MOC_LITERAL(7, 77, 5), // "state"
QT_MOC_LITERAL(8, 83, 8), // "OnSaveAs"
QT_MOC_LITERAL(9, 92, 13), // "OnSendMessage"
QT_MOC_LITERAL(10, 106, 7), // "cString"
QT_MOC_LITERAL(11, 114, 7) // "message"

    },
    "cCameraCalibrationWidget\0SendSaveAs\0"
    "\0filename\0OnNewImage\0newImage\0OnSetState\0"
    "state\0OnSaveAs\0OnSendMessage\0cString\0"
    "message"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_cCameraCalibrationWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   39,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    1,   42,    2, 0x0a /* Public */,
       6,    1,   45,    2, 0x0a /* Public */,
       8,    0,   48,    2, 0x0a /* Public */,
       9,    1,   49,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,

 // slots: parameters
    QMetaType::Void, QMetaType::QImage,    5,
    QMetaType::Void, QMetaType::Int,    7,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 10,   11,

       0        // eod
};

void cCameraCalibrationWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        cCameraCalibrationWidget *_t = static_cast<cCameraCalibrationWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->SendSaveAs((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->OnNewImage((*reinterpret_cast< const QImage(*)>(_a[1]))); break;
        case 2: _t->OnSetState((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->OnSaveAs(); break;
        case 4: _t->OnSendMessage((*reinterpret_cast< cString(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (cCameraCalibrationWidget::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&cCameraCalibrationWidget::SendSaveAs)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject cCameraCalibrationWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_cCameraCalibrationWidget.data,
      qt_meta_data_cCameraCalibrationWidget,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *cCameraCalibrationWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *cCameraCalibrationWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_cCameraCalibrationWidget.stringdata0))
        return static_cast<void*>(const_cast< cCameraCalibrationWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int cCameraCalibrationWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void cCameraCalibrationWidget::SendSaveAs(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
