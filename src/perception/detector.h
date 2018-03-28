#pragma once

#include <iostream>
#include <fstream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <types/types.hpp>

//abstract class for object detector
class Detector{
public:
  Detector();

  //this function performs object detection
  virtual void compute(const cv::Mat& rgb_image_,
                       const cv::Mat& raw_depth_image_) = 0;
  
  //this function returns the output of the detection
  inline const DetectionVector& detections() const {return _detections;}
  
private:  

  //detected objects
  DetectionVector _detections;
};

