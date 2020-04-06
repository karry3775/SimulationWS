/****************************************************************************
** Meta object code from reading C++ file 'aerialmap_display.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/autonomous_vehicle/catvehicle_basics/rviz_satellite/src/aerialmap_display.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'aerialmap_display.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_rviz__AerialMapDisplay[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      24,   23,   23,   23, 0x09,
      46,   23,   23,   23, 0x09,
      60,   23,   23,   23, 0x09,
      74,   23,   23,   23, 0x09,
      88,   23,   23,   23, 0x09,
     106,   23,   23,   23, 0x09,
     124,   23,   23,   23, 0x09,
     137,   23,   23,   23, 0x09,
     152,   23,   23,   23, 0x09,
     184,  176,   23,   23, 0x09,
     218,  176,   23,   23, 0x09,
     249,   23,   23,   23, 0x09,
     279,  267,   23,   23, 0x09,

       0        // eod
};

static const char qt_meta_stringdata_rviz__AerialMapDisplay[] = {
    "rviz::AerialMapDisplay\0\0updateDynamicReload()\0"
    "updateAlpha()\0updateTopic()\0updateFrame()\0"
    "updateDrawUnder()\0updateObjectURI()\0"
    "updateZoom()\0updateBlocks()\0"
    "updateFrameConvention()\0request\0"
    "initiatedRequest(QNetworkRequest)\0"
    "receivedImage(QNetworkRequest)\0"
    "finishedLoading()\0description\0"
    "errorOcurred(QString)\0"
};

void rviz::AerialMapDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        AerialMapDisplay *_t = static_cast<AerialMapDisplay *>(_o);
        switch (_id) {
        case 0: _t->updateDynamicReload(); break;
        case 1: _t->updateAlpha(); break;
        case 2: _t->updateTopic(); break;
        case 3: _t->updateFrame(); break;
        case 4: _t->updateDrawUnder(); break;
        case 5: _t->updateObjectURI(); break;
        case 6: _t->updateZoom(); break;
        case 7: _t->updateBlocks(); break;
        case 8: _t->updateFrameConvention(); break;
        case 9: _t->initiatedRequest((*reinterpret_cast< QNetworkRequest(*)>(_a[1]))); break;
        case 10: _t->receivedImage((*reinterpret_cast< QNetworkRequest(*)>(_a[1]))); break;
        case 11: _t->finishedLoading(); break;
        case 12: _t->errorOcurred((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData rviz::AerialMapDisplay::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject rviz::AerialMapDisplay::staticMetaObject = {
    { &Display::staticMetaObject, qt_meta_stringdata_rviz__AerialMapDisplay,
      qt_meta_data_rviz__AerialMapDisplay, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &rviz::AerialMapDisplay::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *rviz::AerialMapDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *rviz::AerialMapDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__AerialMapDisplay))
        return static_cast<void*>(const_cast< AerialMapDisplay*>(this));
    return Display::qt_metacast(_clname);
}

int rviz::AerialMapDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = Display::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
