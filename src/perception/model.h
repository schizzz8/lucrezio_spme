#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace lucrezio_spme{

  class Model;
  typedef std::vector<Model> ModelVector;

  class Model{
  public:
    Model(std::string type_ = "",
          Eigen::Isometry3f pose_ = Eigen::Isometry3f::Identity(),
          Eigen::Vector3f min_ = Eigen::Vector3f::Zero(),
          Eigen::Vector3f max_ = Eigen::Vector3f::Zero()):
      _type(type_),
      _pose(pose_),
      _min(min_),
      _max(max_){}
    std::string _type;
    Eigen::Isometry3f _pose;
    Eigen::Vector3f _min;
    Eigen::Vector3f _max;
  };

  typedef std::pair<Eigen::Vector3f,Eigen::Vector3f> BoundingBox3D;
  typedef std::vector<BoundingBox3D> BoundingBox3DVector;

}
