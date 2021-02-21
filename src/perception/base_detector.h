#pragma once

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "detection.h"

namespace lucrezio_spme{

  //abstract class for object detector
  class BaseDetector{
  public:

    //constructor
    BaseDetector() {}

    //destructor
    virtual ~BaseDetector() {}

    //this function performs object detection
    virtual DetectionVector compute(const cv::Mat &rgb_image_,
                                    const cv::Mat &raw_depth_image_) = 0;
  };

}
