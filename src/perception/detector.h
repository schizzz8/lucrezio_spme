#pragma once

#include <iostream>
#include <fstream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <detection.h>

class Detector{
public:
  Detector();

  void setImages(const srrg_core::RGBImage& rgb_image_,
                 const srrg_core::RawDepthImage& raw_depth_image_);
  
  virtual void compute() = 0;
  
  inline const Detections& detections() const {return _detections;}
  
private:
  cv::Mat _rgb_image;
  cv::Mat _raw_depth_image;
  
  Detections _detections;
};

