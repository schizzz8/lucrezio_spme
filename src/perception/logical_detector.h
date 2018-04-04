#pragma once

#include "base_detector.h"
#include "detection.h"
#include "model.h"

#include <srrg_types/types.hpp>
#include <srrg_image_utils/depth_utils.h>
#include <srrg_image_utils/point_image_utils.h>

namespace lucrezio_spme{

  //this class implements an object detector in a simulation environment
  class LogicalDetector : public BaseDetector{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //specialized compute method
    DetectionVector compute(const cv::Mat &rgb_image_, const cv::Mat &raw_depth_image_);

    //setters and getters
    inline void setK(const Eigen::Matrix3f& K_){_K = K_;}
    inline void setTransform(const Eigen::Isometry3f &transform_){_transform = transform_;}
    inline void setModels(const ModelVector &models_){_models = models_; _bounding_boxes.resize(_models.size());}

  protected:

    //rgbd camera matrix
    Eigen::Matrix3f _K;

    //model to rgbd camera transform
    Eigen::Isometry3f _transform;

    //vector of models detected by the logical camera
    ModelVector _models;

    //models 3D bounding boxes
    BoundingBox3DVector _bounding_boxes;

    //organized point cloud obtained from the depth image
    srrg_core::Float3Image _points_image;

    //output image, each pixel stores the label of the corresponding object (for visualization only)
    srrg_core::RGBImage _label_image;

  private:

    //for each model the 3d bounding box is transformed in the rgbd camera frame
    void transformBoundingBoxes(DetectionVector &detections);

    //check if a point falls into a bounding box
    inline bool inRange(const Eigen::Vector3f &point, const BoundingBox3D &bounding_box){
      return (point.x() >= bounding_box.first.x()-0.01 && point.x() <= bounding_box.second.x()+0.01 &&
              point.y() >= bounding_box.first.y()-0.01 && point.y() <= bounding_box.second.y()+0.01 &&
              point.z() >= bounding_box.first.z()-0.01 && point.z() <= bounding_box.second.z()+0.01);
    }

    //performs the actual object detection
    void computeImageBoundingBoxes(DetectionVector &detections);

    //functions to draw label_image
    cv::Vec3b type2color(std::string type);
    void computeLabelImage(const DetectionVector &detections);

  };
}
