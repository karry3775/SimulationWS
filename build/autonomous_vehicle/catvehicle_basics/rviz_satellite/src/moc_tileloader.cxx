/****************************************************************************
** Meta object code from reading C++ file 'tileloader.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/autonomous_vehicle/catvehicle_basics/rviz_satellite/src/tileloader.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'tileloader.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_TileLoader[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: signature, parameters, type, tag, flags
      20,   12,   11,   11, 0x05,
      54,   12,   11,   11, 0x05,
      85,   11,   11,   11, 0x05,
     115,  103,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
     143,  137,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_TileLoader[] = {
    "TileLoader\0\0request\0"
    "initiatedRequest(QNetworkRequest)\0"
    "receivedImage(QNetworkRequest)\0"
    "finishedLoading()\0description\0"
    "errorOcurred(QString)\0reply\0"
    "finishedRequest(QNetworkReply*)\0"
};

void TileLoader::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        TileLoader *_t = static_cast<TileLoader *>(_o);
        switch (_id) {
        case 0: _t->initiatedRequest((*reinterpret_cast< QNetworkRequest(*)>(_a[1]))); break;
        case 1: _t->receivedImage((*reinterpret_cast< QNetworkRequest(*)>(_a[1]))); break;
        case 2: _t->finishedLoading(); break;
        case 3: _t->errorOcurred((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 4: _t->finishedRequest((*reinterpret_cast< QNetworkReply*(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData TileLoader::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject TileLoader::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_TileLoader,
      qt_meta_data_TileLoader, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &TileLoader::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *TileLoader::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *TileLoader::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_TileLoader))
        return static_cast<void*>(const_cast< TileLoader*>(this));
    return QObject::qt_metacast(_clname);
}

int TileLoader::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void TileLoader::initiatedRequest(QNetworkRequest _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void TileLoader::receivedImage(QNetworkRequest _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void TileLoader::finishedLoading()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}

// SIGNAL 3
void TileLoader::errorOcurred(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_END_MOC_NAMESPACE
