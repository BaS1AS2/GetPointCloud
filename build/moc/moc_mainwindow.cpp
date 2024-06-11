/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.14.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.14.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[20];
    char stringdata0[403];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 13), // "image_capture"
QT_MOC_LITERAL(2, 25, 0), // ""
QT_MOC_LITERAL(3, 26, 22), // "on_open_camera_clicked"
QT_MOC_LITERAL(4, 49, 23), // "on_close_camera_clicked"
QT_MOC_LITERAL(5, 73, 20), // "on_get_image_clicked"
QT_MOC_LITERAL(6, 94, 20), // "on_imageShow_clicked"
QT_MOC_LITERAL(7, 115, 25), // "on_get_centerline_clicked"
QT_MOC_LITERAL(8, 141, 22), // "on_get_ptcloud_clicked"
QT_MOC_LITERAL(9, 164, 10), // "read_angle"
QT_MOC_LITERAL(10, 175, 16), // "capture_rotation"
QT_MOC_LITERAL(11, 192, 21), // "move_controll_connect"
QT_MOC_LITERAL(12, 214, 26), // "on_get_calibration_clicked"
QT_MOC_LITERAL(13, 241, 27), // "on_capture_rotation_clicked"
QT_MOC_LITERAL(14, 269, 28), // "on_splicing_rotation_clicked"
QT_MOC_LITERAL(15, 298, 9), // "read_move"
QT_MOC_LITERAL(16, 308, 14), // "capture_linear"
QT_MOC_LITERAL(17, 323, 25), // "on_capture_linear_clicked"
QT_MOC_LITERAL(18, 349, 26), // "on_splicing_linear_clicked"
QT_MOC_LITERAL(19, 376, 26) // "on_btn_calibration_clicked"

    },
    "MainWindow\0image_capture\0\0"
    "on_open_camera_clicked\0on_close_camera_clicked\0"
    "on_get_image_clicked\0on_imageShow_clicked\0"
    "on_get_centerline_clicked\0"
    "on_get_ptcloud_clicked\0read_angle\0"
    "capture_rotation\0move_controll_connect\0"
    "on_get_calibration_clicked\0"
    "on_capture_rotation_clicked\0"
    "on_splicing_rotation_clicked\0read_move\0"
    "capture_linear\0on_capture_linear_clicked\0"
    "on_splicing_linear_clicked\0"
    "on_btn_calibration_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      18,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,  104,    2, 0x08 /* Private */,
       3,    0,  105,    2, 0x08 /* Private */,
       4,    0,  106,    2, 0x08 /* Private */,
       5,    0,  107,    2, 0x08 /* Private */,
       6,    0,  108,    2, 0x08 /* Private */,
       7,    0,  109,    2, 0x08 /* Private */,
       8,    0,  110,    2, 0x08 /* Private */,
       9,    0,  111,    2, 0x08 /* Private */,
      10,    0,  112,    2, 0x08 /* Private */,
      11,    0,  113,    2, 0x08 /* Private */,
      12,    0,  114,    2, 0x08 /* Private */,
      13,    0,  115,    2, 0x08 /* Private */,
      14,    0,  116,    2, 0x08 /* Private */,
      15,    0,  117,    2, 0x08 /* Private */,
      16,    0,  118,    2, 0x08 /* Private */,
      17,    0,  119,    2, 0x08 /* Private */,
      18,    0,  120,    2, 0x08 /* Private */,
      19,    0,  121,    2, 0x08 /* Private */,

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
    QMetaType::Bool,
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

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->image_capture(); break;
        case 1: _t->on_open_camera_clicked(); break;
        case 2: _t->on_close_camera_clicked(); break;
        case 3: _t->on_get_image_clicked(); break;
        case 4: _t->on_imageShow_clicked(); break;
        case 5: _t->on_get_centerline_clicked(); break;
        case 6: _t->on_get_ptcloud_clicked(); break;
        case 7: _t->read_angle(); break;
        case 8: _t->capture_rotation(); break;
        case 9: { bool _r = _t->move_controll_connect();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 10: _t->on_get_calibration_clicked(); break;
        case 11: _t->on_capture_rotation_clicked(); break;
        case 12: _t->on_splicing_rotation_clicked(); break;
        case 13: _t->read_move(); break;
        case 14: _t->capture_linear(); break;
        case 15: _t->on_capture_linear_clicked(); break;
        case 16: _t->on_splicing_linear_clicked(); break;
        case 17: _t->on_btn_calibration_clicked(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject MainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_MainWindow.data,
    qt_meta_data_MainWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 18)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 18;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 18)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 18;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
