#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

class Object;
typedef std::vector<Object*> Objects;
typedef std::pair<Object,Object> Association;

class Object {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Object(int id_=-1,
         std::string type_="",
         Eigen::Isometry3f pose_=Eigen::Isometry3f::Identity(),
         Eigen::Vector3f min_=Eigen::Vector3f::Zero(),
         Eigen::Vector3f max_=Eigen::Vector3f::Zero());

private:
  int _id;
  std::string _type;
  Eigen::Isometry3f _pose;
  Eigen::Vector3f _min;
  Eigen::Vector3f _max;
};

