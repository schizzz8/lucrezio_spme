#pragma once

#include "base_mapper.h"

#include <iostream>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <srrg_types/types.hpp>


namespace lucrezio_spme{
  class SemanticMapper : public BaseMapper{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SemanticMapper();

    //set camera matrix
    inline void setK(const Eigen::Matrix3f& K_){_K = K_; _invK = _K.inverse();}

    //specialized extractObjects method
    void extractObjects(const DetectionVector &detections,
                        const cv::Mat &depth_image_);

    //specialized findAssociations method
    void findAssociations();

    //specialized mergeMaps method
    void mergeMaps();

  protected:
    bool _local_set;
    bool _global_set;

    ObjectPtrPairVector _associations;
  private:

    //rgbd camera parameters
    float _raw_depth_scale;
    float _min_distance, _max_distance;

    //rgbd camera matrix
    Eigen::Matrix3f _K,_invK;

    //computes 3d coordinates for each pixel
    srrg_core::Vector3fVector unproject(const std::vector<Eigen::Vector2i> &pixels,
                                        const cv::Mat &depth_image_);

    //given a point cloud, computes 3d bounding box
    void getLowerUpper3d(const srrg_core::Vector3fVector &points, Eigen::Vector3f &lower, Eigen::Vector3f &upper);

    //given an object in the local map, returns the id of the corresponding object in the global map
    int associationID(const ObjectPtr &local);
  };
}
