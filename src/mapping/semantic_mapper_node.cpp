#include "semantic_mapper_node.h"

namespace lucrezio_spme{

  using namespace srrg_core;

  SemanticMapperNode::SemanticMapperNode(ros::NodeHandle nh_):
    SemanticMapper(),
    _nh(nh_),
    _image_bounding_boxes_sub(_nh,"/image_bounding_boxes",100),
    _depth_image_sub(_nh,"/camera/depth/image_raw", 100),
    _synchronizer(FilterSyncPolicy(100),_image_bounding_boxes_sub,_depth_image_sub){

    _got_info = false;
    _camera_info_sub = _nh.subscribe("/camera/depth/camera_info",
                                     1000,
                                     &SemanticMapperNode::cameraInfoCallback,
                                     this);

    _synchronizer.registerCallback(boost::bind(&SemanticMapperNode::filterCallback, this, _1, _2));

    _shape = visualization_msgs::Marker::CUBE;
    _markers_pub = _nh.advertise<visualization_msgs::MarkerArray>("visualization_markers",1);
  }

  void SemanticMapperNode::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &camera_info_msg){
    sensor_msgs::CameraInfo camerainfo;
    camerainfo.K = camera_info_msg->K;

    Eigen::Matrix3f K;

    ROS_INFO("Got camera info!");
    K(0,0) = camerainfo.K.c_array()[0];
    K(0,1) = camerainfo.K.c_array()[1];
    K(0,2) = camerainfo.K.c_array()[2];
    K(1,0) = camerainfo.K.c_array()[3];
    K(1,1) = camerainfo.K.c_array()[4];
    K(1,2) = camerainfo.K.c_array()[5];
    K(2,0) = camerainfo.K.c_array()[6];
    K(2,1) = camerainfo.K.c_array()[7];
    K(2,2) = camerainfo.K.c_array()[8];
    std::cerr << K << std::endl;

    setK(K);

    _got_info = true;
    _camera_info_sub.shutdown();
  }

  void SemanticMapperNode::filterCallback(const ImageBoundingBoxesArray::ConstPtr &image_bounding_boxes_msg,
                                          const sensor_msgs::Image::ConstPtr &depth_image_msg){

    if(_got_info && !image_bounding_boxes_msg->image_bounding_boxes.empty()){

      //Listen to camera pose
      tf::StampedTransform depth_camera_pose;
      try {
        _listener.waitForTransform("map",
                                   "camera_depth_optical_frame",
                                   ros::Time(0),
                                   ros::Duration(3));
        _listener.lookupTransform("map",
                                  "camera_depth_optical_frame",
                                  ros::Time(0),
                                  depth_camera_pose);
      }
      catch(tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
      }

      //set robot pose
      Eigen::Isometry3f globalT=tfTransform2eigen(depth_camera_pose);
      setGlobalT(globalT);

      //Extract depth image from ROS message
      cv_bridge::CvImageConstPtr depth_cv_ptr;
      try{
        depth_cv_ptr = cv_bridge::toCvShare(depth_image_msg);
      } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
      cv::Mat depth_image = depth_cv_ptr->image.clone();

      //extract objects
      DetectionVector detections = imageBoundingBoxes2Detections(image_bounding_boxes_msg);
      extractObjects(detections,depth_image);

      //data association
      findAssociations();

      //merging
      mergeMaps();

      //publish global map (to be visualized with RViz)
      if(_global_map->size() && _markers_pub.getNumSubscribers() > 0){
          visualization_msgs::MarkerArray markers;

          for(int i=0; i < _global_map->size(); ++i){
              visualization_msgs::Marker marker;
              const ObjectPtr& object = (*_global_map)[i];
              makeMarkerFromObject(marker,object);
              markers.markers.push_back(marker);
          }
          _markers_pub.publish(markers);
      }
    }
  }

  Eigen::Isometry3f SemanticMapperNode::tfTransform2eigen(const tf::Transform &p){
    Eigen::Isometry3f iso;
    iso.translation().x()=p.getOrigin().x();
    iso.translation().y()=p.getOrigin().y();
    iso.translation().z()=p.getOrigin().z();
    Eigen::Quaternionf q;
    tf::Quaternion tq = p.getRotation();
    q.x()= tq.x();
    q.y()= tq.y();
    q.z()= tq.z();
    q.w()= tq.w();
    iso.linear()=q.toRotationMatrix();
    return iso;
  }

  DetectionVector SemanticMapperNode::imageBoundingBoxes2Detections(const ImageBoundingBoxesArray::ConstPtr &image_bounding_boxes_msg){
    const std::vector<ImageBoundingBox>& image_bounding_boxes = image_bounding_boxes_msg->image_bounding_boxes;
    DetectionVector detections;
    std::vector<Eigen::Vector2i> pixels;
    for(int i=0; i < image_bounding_boxes.size(); ++i){
      const ImageBoundingBox& image_bounding_box = image_bounding_boxes[i];
      std::string type (image_bounding_box.type);
      Eigen::Vector2i top_left (image_bounding_box.top_left.r,image_bounding_box.top_left.c);
      Eigen::Vector2i bottom_right(image_bounding_box.bottom_right.r,image_bounding_box.bottom_right.c);
      pixels.clear();
      for(int j=0; j < image_bounding_box.pixels.size(); ++j){
        pixels.push_back(Eigen::Vector2i(image_bounding_box.pixels[j].r,image_bounding_box.pixels[j].c));
      }
      detections.push_back(DetectionPtr (new Detection(type,top_left,bottom_right,pixels)));
    }
    return detections;
  }

  void SemanticMapperNode::makeMarkerFromObject(visualization_msgs::Marker &marker, const ObjectPtr & object){
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = object->id();
    marker.type = _shape;
    marker.action = visualization_msgs::Marker::ADD;

    const Eigen::Vector3f centroid = (object->max()+object->min())*0.5f;
    const Eigen::Vector3f half_size = (object->max()-object->min())*0.5f;

    marker.pose.position.x = centroid.x();
    marker.pose.position.y = centroid.y();
    marker.pose.position.z = centroid.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = half_size.x();
    marker.scale.y = half_size.y();
    marker.scale.z = half_size.z();

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
}
}
