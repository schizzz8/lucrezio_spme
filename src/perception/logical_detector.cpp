#include "logical_detector.h"

namespace lucrezio_spme{

  using namespace srrg_core;


  void LogicalDetector::transformBoundingBoxes(DetectionVector &detections){

    for(int i=0; i<_models.size(); ++i){

      const Model &model = _models[i];
      const Eigen::Isometry3f& model_pose=model._pose;

      std::vector<Eigen::Vector3f> points;
      points.push_back(_transform*model_pose*Eigen::Vector3f(model._min.x(),model._min.y(),model._min.z()));
      points.push_back(_transform*model_pose*Eigen::Vector3f(model._max.x(),model._max.y(),model._max.z()));

      float x_min=100000,x_max=-100000,y_min=100000,y_max=-100000,z_min=100000,z_max=-100000;
      for(int i=0; i < 2; ++i){
        if(points[i].x()<x_min)
          x_min = points[i].x();
        if(points[i].x()>x_max)
          x_max = points[i].x();
        if(points[i].y()<y_min)
          y_min = points[i].y();
        if(points[i].y()>y_max)
          y_max = points[i].y();
        if(points[i].z()<z_min)
          z_min = points[i].z();
        if(points[i].z()>z_max)
          z_max = points[i].z();
      }
      _bounding_boxes[i] = std::make_pair(Eigen::Vector3f(x_min,y_min,z_min),Eigen::Vector3f(x_max,y_max,z_max));
      detections[i].type() = model._type;
    }
  }

  void LogicalDetector::computeImageBoundingBoxes(DetectionVector &detections){
    for(int r=0; r<_points_image.rows; ++r){
      const cv::Vec3f* point_ptr=_points_image.ptr<const cv::Vec3f>(r);
      for(int c=0; c<_points_image.cols; ++c, ++point_ptr){
        const cv::Vec3f& p = *point_ptr;

        if(cv::norm(p) < 1e-3)
          continue;

        const Eigen::Vector3f point(p[0],p[1],p[2]);

        for(int j=0; j < _bounding_boxes.size(); ++j){

          if(inRange(point,_bounding_boxes[j])){
            if(r < detections[j].topLeft().x())
              detections[j].topLeft().x() = r;
            if(r > detections[j].bottomRight().x())
              detections[j].bottomRight().x() = r;

            if(c < detections[j].topLeft().y())
              detections[j].topLeft().y() = c;
            if(c > detections[j].bottomRight().y())
              detections[j].bottomRight().y() = c;

            detections[j].pixels()[detections[j].size()] = Eigen::Vector2i(r,c);
            ++(detections[j].size());
            break;
          }
        }


      }
    }

    for(size_t i=0; i < detections.size(); ++i){
      detections[i].pixels().resize(detections[i].size());
    }
  }

  DetectionVector LogicalDetector::compute(const cv::Mat &rgb_image_,
                                           const cv::Mat &raw_depth_image_){

    int rows=rgb_image_.rows;
    int cols=rgb_image_.cols;

    //compute points image
    srrg_core::Float3Image directions_image;
    directions_image.create(rows,cols);
    initializePinholeDirections(directions_image,_K);
    _points_image.create(rows,cols);
    computePointsImage(_points_image,
                       directions_image,
                       raw_depth_image_,
                       0.02f,
                       8.0f);

    //initialize detection vector
    int num_models = _models.size();
    DetectionVector detections(num_models);

    //Compute world bounding boxes
    double cv_wbb_time = (double)cv::getTickCount();
    transformBoundingBoxes(detections);
    printf("Computing WBB took: %f\n",((double)cv::getTickCount() - cv_wbb_time)/cv::getTickFrequency());

    //Compute image bounding boxes
    double cv_ibb_time = (double)cv::getTickCount();
    computeImageBoundingBoxes(detections);
    printf("Computing IBB took: %f\n",((double)cv::getTickCount() - cv_ibb_time)/cv::getTickFrequency());

    //Compute label image (for visualization only)
    _label_image.create(rows,cols);
    _label_image=cv::Vec3b(0,0,0);
    computeLabelImage(detections);

    return detections;
  }

  cv::Vec3b LogicalDetector::type2color(std::string type){
    int c;

    if(type == "table")
      c = 1;
    if(type == "tomato")
      c = 2;
    if(type == "salt")
      c = 3;
    if(type == "milk")
      c = 4;

    std::stringstream stream;
    stream << std::setw(6) << std::setfill('0') << std::hex << ((float)c/4.0f)*16777215;
    std::string result(stream.str());

    unsigned long r_value = std::strtoul(result.substr(0,2).c_str(), 0, 16);
    unsigned long g_value = std::strtoul(result.substr(2,2).c_str(), 0, 16);
    unsigned long b_value = std::strtoul(result.substr(4,2).c_str(), 0, 16);

    cv::Vec3b color(r_value,g_value,b_value);
    return color;
  }

  void LogicalDetector::computeLabelImage(const DetectionVector &detections){
    for(int i=0; i < detections.size(); ++i){
      std::string type = detections[i].type();
      std::string cropped_type = type.substr(0,type.find_first_of("_"));
      cv::Vec3b color = type2color(cropped_type);
      for(int j=0; j < detections[i].pixels().size(); ++j){
        int r = detections[i].pixels()[j].x();
        int c = detections[i].pixels()[j].y();

        _label_image.at<cv::Vec3b>(c,r) = color;
      }
    }
  }
}
