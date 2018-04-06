#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace lucrezio_spme {

  class Detection;
  typedef std::vector<Detection> DetectionVector;

  //this class is a container for the output of an object detector
  class Detection{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //constructor
    Detection(const std::string& type_="",
              const Eigen::Vector2i& top_left_ = Eigen::Vector2i(10000,10000),
              const Eigen::Vector2i& bottom_right_ = Eigen::Vector2i(-10000,-10000),
              const std::vector<Eigen::Vector2i>& pixels_ = std::vector<Eigen::Vector2i>(640*480)):
      _type(type_),
      _top_left(top_left_),
      _bottom_right(bottom_right_),
      _pixels(pixels_),
      _size(0){}

    //setters and getters
    inline const std::string &type() const {return _type;}
    inline std::string &type() {return _type;}
    inline const Eigen::Vector2i &topLeft() const {return _top_left;}
    inline Eigen::Vector2i &topLeft() {return _top_left;}
    inline const Eigen::Vector2i &bottomRight() const {return _bottom_right;}
    inline Eigen::Vector2i &bottomRight() {return _bottom_right;}
    inline const std::vector<Eigen::Vector2i> &pixels() const {return _pixels;}
    inline std::vector<Eigen::Vector2i>& pixels() {return _pixels;}
    inline const int size() const {return _size;}
    inline int &size() {return _size;}

  private:
    //semantic class of the detected object
    std::string _type;

    //top left pixel of the image bounding box
    Eigen::Vector2i _top_left;

    //bottom right pixel of the image bounding box
    Eigen::Vector2i _bottom_right;

    //array of pixels that belong to the detected object
    std::vector<Eigen::Vector2i> _pixels;

    //size of pixels array
    int _size;
  };

}
