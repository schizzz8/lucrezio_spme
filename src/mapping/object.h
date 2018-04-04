#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace lucrezio_spme{

  class Object;
  typedef std::shared_ptr<Object> ObjectPtr;
  typedef std::vector<ObjectPtr> ObjectPtrVector;
  typedef std::pair<ObjectPtr,ObjectPtr> ObjectPtrPair;
  typedef std::vector<ObjectPtrPair> ObjectPtrPairVector;

  //this class is a container for a 3d object that composes the semantic map
  class Object {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //constructor
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

    //setters and getters
    inline const int id() const {return _id;}
    inline int& id() {return _id;}
    inline const std::string& type() const {return _type;}
    inline std::string& type() {return _type;}
    inline const Eigen::Isometry3f& pose() const {return _pose;}
    inline Eigen::Isometry3f& pose() {return _pose;}
    inline const Eigen::Vector3f& min() const {return _min;}
    inline Eigen::Vector3f& min() {return _min;}
    inline const Eigen::Vector3f& max() const {return _max;}
    inline Eigen::Vector3f& max() {return _max;}

    //overriden operators
    bool operator < (const Object &o);
    bool operator == (const Object &o);

    //merge two objects
    void merge(const ObjectPtr &o);

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
}
