#include "object.h"

namespace lucrezio_spme{

  using namespace std;

  bool Object::operator <(const Object &o){
    return (this->_id < o.id());
  }

  bool Object::operator ==(const Object &o){
    return (this->_id == o.id());
  }

  void Object::merge(const ObjectPtr & o){
    if(o->lower().x() < _min.x())
      _min.x() = o->lower().x();
    if(o->upper().x() > _max.x())
      _max.x() = o->upper().x();
    if(o->lower().y() < _min.y())
      _min.y() = o->lower().y();
    if(o->upper().y() > _max.y())
      _max.y() = o->upper().y();
    if(o->lower().z() < _min.z())
      _min.z() = o->lower().z();
    if(o->upper().z() > _max.z())
      _max.z() = o->upper().z();

    _pose.translation() = (_min+_max)/2.0f;

  }


}

