#pragma once

//this class is a container for a 3d object that composes the semantic map
class Object {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Object(int id_=-1,
         std::string type_="",
         Eigen::Isometry3f pose_=Eigen::Isometry3f::Identity(),
         Eigen::Vector3f min_=Eigen::Vector3f::Zero(),
         Eigen::Vector3f max_=Eigen::Vector3f::Zero()):
    _id(id_),
    _type(type_),
    _pose(pose_),
    _min(min_),
    _max(max_){}


private:

  //unique identifier (in the map) of the object
  int _id;

  //semantic class of the object
  std::string _type;

  //3D pose of the object
  Eigen::Isometry3f _pose;

  //lower vertex of the object bounding box
  Eigen::Vector3f _min;

  //upper vertex of the object bounding box
  Eigen::Vector3f _max;
};

