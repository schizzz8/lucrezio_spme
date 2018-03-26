#include "object.h"

Object::Object(int id_,
               std::string type_,
               Eigen::Isometry3f pose_,
               Eigen::Vector3f min_,
               Eigen::Vector3f max_):
  _id(id_),
  _type(type_),
  _pose(pose_),
  _min(min_),
  _max(max_){

}
