#include "semantic_mapper.h"

namespace lucrezio_spme{

  using namespace srrg_core;

  SemanticMapper::SemanticMapper():
    BaseMapper(){
    _raw_depth_scale = 0.001;
    _min_distance = 0.02;
    _max_distance = 8.0;

    _local_set = false;
    _global_set = false;

    _globalT = Eigen::Isometry3f::Identity();
  }

  Vector3fVector SemanticMapper::unproject(const std::vector<Eigen::Vector2i> &pixels,
                                           const cv::Mat &depth_image_){

    int num_pixels = pixels.size();
    std::cerr << "num pixels: " << num_pixels << std::endl;
    Vector3fVector points(num_pixels);

    int k=0;
    for(int idx=0; idx < num_pixels; ++idx){
      const Eigen::Vector2i& pixel = pixels[idx];
      int r = pixel.x();
      int c = pixel.y();
      const unsigned short& depth = depth_image_.at<const unsigned short>(r,c);
      float d = depth * _raw_depth_scale;

      if(d <= _min_distance)
        continue;

      if(d >= _max_distance)
        continue;

      Eigen::Vector3f camera_point = _invK * Eigen::Vector3f(c*d,r*d,d);
      Eigen::Vector3f map_point = _globalT*camera_point;

      points[k]=map_point;
      k++;
    }
    points.resize(k);
    return points;
  }

  void SemanticMapper::getLowerUpper3d(const Vector3fVector &points, Eigen::Vector3f &lower, Eigen::Vector3f &upper){
    lower.x() = std::numeric_limits<float>::max();
    lower.y() = std::numeric_limits<float>::max();
    lower.z() = std::numeric_limits<float>::max();
    upper.x() = -std::numeric_limits<float>::max();
    upper.y() = -std::numeric_limits<float>::max();
    upper.z() = -std::numeric_limits<float>::max();

    for(int i=0; i < points.size(); ++i){

      if(points[i].x() < lower.x())
        lower.x() = points[i].x();
      if(points[i].x() > upper.x())
        upper.x() = points[i].x();
      if(points[i].y() < lower.y())
        lower.y() = points[i].y();
      if(points[i].y() > upper.y())
        upper.y() = points[i].y();
      if(points[i].z() < lower.z())
        lower.z() = points[i].z();
      if(points[i].z() > upper.z())
        upper.z() = points[i].z();
    }
  }

  void SemanticMapper::extractObjects(const DetectionVector &detections,
                                      const cv::Mat &depth_image_){
    bool populate_global = false;
    if(!_global_set){
      populate_global = true;
      _global_set = true;
    } else {
      _local_map->clear();
      _local_set = true;
    }

    for(int i=0; i < detections.size(); ++i){

      const Detection& detection = detections[i];

      if((detection.bottomRight()-detection.topLeft()).norm() < 1e-3 ||
         (detection.bottomRight()-detection.topLeft()).norm() >= 2e+4)
        continue;

      std::cerr << std::endl << detection.type() << ": [(";
      std::cerr << detection.topLeft().transpose() << ") - (" << detection.bottomRight().transpose() << ")]" << std::endl;

      std::string object_type = detection.type().substr(0,detection.type().find_first_of("_"));

      Vector3fVector points = unproject(detection.pixels(),depth_image_);
      Eigen::Vector3f lower,upper;
      getLowerUpper3d(points,lower,upper);
      std::cerr << "BB: [(" << lower.transpose() << "," << upper.transpose() << ")]" << std::endl;

      Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
      pose.translation() = (upper+lower)/2.0f;

      ObjectPtr obj_ptr = ObjectPtr(new Object(i,
                                               object_type,
                                               pose,
                                               lower,
                                               upper));

      if(populate_global)
        _global_map->addObject(obj_ptr);
      else
        _local_map->addObject(obj_ptr);
    }
  }

  void SemanticMapper::findAssociations(){
    if(!_global_set || !_local_set)
      return;

    const int local_size = _local_map->size();
    const int global_size = _global_map->size();

    std::cerr << "[Data Association] ";
    std::cerr << "{Local Map size: " << local_size << "} ";
    std::cerr << "- {Global Map size: " << global_size << "}" << std::endl;

    _associations.clear();

    for(int i=0; i < global_size; ++i){
      const ObjectPtr &global = (*_global_map)[i];
      const std::string &global_type = global->type();

      std::cerr << "\t>> Global: " << global_type;

      ObjectPtr local_best;
      float best_error = std::numeric_limits<float>::max();

      for(int j=0; j < local_size; ++j){
        const ObjectPtr &local = (*_local_map)[j];
        const std::string &local_type = local->type();

        if(local_type != global_type)
          continue;

        Eigen::Vector3f e_c = local->pose().translation() - global->pose().translation();

        float error = e_c.transpose()*e_c;

        if(error<best_error){
          best_error = error;
          local_best = local;
        }
      }
      if(local_best->type() == "")
        continue;

      std::cerr << " - Local ID: " << local_best->type() << std::endl;

      _associations.push_back(ObjectPtrPair(global,local_best));
    }
  }

  int SemanticMapper::associationID(const ObjectPtr &local){
    for(int i=0; i < _associations.size(); ++i)
      if(_associations[i].second->id() == local->id())
        return _associations[i].first->id();
    return -1;
  }

  void SemanticMapper::mergeMaps(){
    if(!_global_set || !_local_set)
      return;

    int added = 0, merged = 0;
    for(int i=0; i < _local_map->size(); ++i){
      const ObjectPtr &local = (*_local_map)[i];
      int association_id = associationID(local);
      if(association_id == -1){
        _global_map->addObject(local);
        added++;
        continue;
      } else {
        ObjectPtr &global_associated = (*_global_map)[association_id];

        if(local->type() != global_associated->type())
          continue;

        global_associated->merge(local);
        merged++;
      }
    }
  }
}
