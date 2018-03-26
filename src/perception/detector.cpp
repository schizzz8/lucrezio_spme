#include "detector.h"

Detector::Detector(){
  _detections.clear();
}

void Detector::setImages(const cv::Mat &rgb_image_, const cv::Mat &raw_depth_image_){
  _rgb_image = rgb_image_;
  _raw_depth_image = raw_depth_image_;
}
