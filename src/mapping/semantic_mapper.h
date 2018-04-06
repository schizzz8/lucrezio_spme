#pragma once

#include "base_mapper.h"

#include <iostream>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <srrg_types/types.hpp>
#include <srrg_image_utils/depth_utils.h>
#include <srrg_image_utils/point_image_utils.h>

namespace lucrezio_spme{
  class SemanticMapper : public BaseMapper{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SemanticMapper();

    //set camera matrix
    inline void setK(const Eigen::Matrix3f& K_){_K = K_; _invK = _K.inverse();}

    //set robot pose
    inline void setGlobalT(const Eigen::Isometry3f &globalT_){_globalT = globalT_;}

    //specialized extractObjects method
    void extractObjects(const DetectionVector &detections,
                        const cv::Mat &depth_image_);

    //specialized findAssociations method
    void findAssociations();

    //specialized mergeMaps method
    void mergeMaps();

  protected:

    //organized point cloud obtained from the depth image
    srrg_core::Float3Image _points_image;

    //point cloud normals
    srrg_core::Float3Image _normals_image;

    //pose of the robot w.r.t. the global map
    Eigen::Isometry3f _globalT;

    //flags
    bool _local_set;
    bool _global_set;

    //vector of associations
    ObjectPtrPairVector _associations;


  private:

    //rgbd camera parameters
    float _raw_depth_scale;
    float _min_distance, _max_distance;

    //rgbd camera matrix
    Eigen::Matrix3f _K,_invK;

    //this function builds an object from the detector output
    ObjectPtr objectFromDetection(const Detection &detection);

    //given an object in the local map, returns the id of the corresponding object in the global map
    int associationID(const ObjectPtr &local);
  };
}
