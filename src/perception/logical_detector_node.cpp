#include "logical_detector_node.h"

namespace lucrezio_spme{

  using namespace srrg_core;

  LogicalDetectorNode::LogicalDetectorNode(ros::NodeHandle nh_):
    _nh(nh_),
    _logical_image_sub(_nh,"/gazebo/logical_camera_image",1),
    _depth_image_sub(_nh,"/camera/depth/image_raw",1),
    _rgb_image_sub(_nh,"/camera/rgb/image_raw", 1),
    _synchronizer(FilterSyncPolicy(10),_logical_image_sub,_depth_image_sub,_rgb_image_sub),
    _it(_nh){

    _got_info = false;
    _camera_info_sub = _nh.subscribe("/camera/depth/camera_info",
                                     1000,
                                     &LogicalDetectorNode::cameraInfoCallback,
                                     this);

    _synchronizer.registerCallback(boost::bind(&LogicalDetectorNode::filterCallback, this, _1, _2, _3));

    _model_state_client = _nh.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");

    _image_bounding_boxes_pub = _nh.advertise<lucrezio_spme::ImageBoundingBoxesArray>("/image_bounding_boxes", 1);
    _label_image_pub = _it.advertise("/camera/rgb/label_image", 1);

    ROS_INFO("Starting logical detector node!");
  }

  void LogicalDetectorNode::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &camera_info_msg){
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

  void LogicalDetectorNode::filterCallback(const lucrezio_simulation_environments::LogicalImage::ConstPtr &logical_image_msg,
                                           const sensor_msgs::Image::ConstPtr &depth_image_msg,
                                           const sensor_msgs::Image::ConstPtr &rgb_image_msg){
    if(_got_info && !logical_image_msg->models.empty()){

      ROS_INFO("--------------------------");
      ROS_INFO("Executing filter callback!");
      ROS_INFO("--------------------------");
      std::cerr << std::endl;

      //request robot pose
      gazebo_msgs::GetModelState model_state;
      model_state.request.model_name = "robot";
      tf::StampedTransform robot_pose;
      if(_model_state_client.call(model_state)){
        ROS_INFO("Received robot model state!");
        tf::poseMsgToTF(model_state.response.pose,robot_pose);
      }else
        ROS_ERROR("Failed to call service gazebo/get_model_state");

      //get rgbd camera transform
      Eigen::Isometry3f rgbd_pose = Eigen::Isometry3f::Identity();
      rgbd_pose.translation() = Eigen::Vector3f(0.0,0.0,0.5);
      rgbd_pose.linear() = Eigen::Quaternionf(0.5,-0.5,0.5,-0.5).toRotationMatrix();
      const Eigen::Isometry3f rgbd_transform=tfTransform2eigen(robot_pose)*rgbd_pose;

      //get logical camera transform
      tf::StampedTransform logical_pose;
      tf::poseMsgToTF(logical_image_msg->pose,logical_pose);
      const Eigen::Isometry3f logical_transform=tfTransform2eigen(logical_pose);

      //set transform
      setTransform(rgbd_transform.inverse()*logical_transform);

      //set models
      ModelVector models = logicalImageToModels(logical_image_msg);
      setModels(models);

      //Extract rgb and depth image from ROS messages
      cv_bridge::CvImageConstPtr rgb_cv_ptr,depth_cv_ptr;
      try{
        rgb_cv_ptr = cv_bridge::toCvShare(rgb_image_msg);
        depth_cv_ptr = cv_bridge::toCvShare(depth_image_msg);
      } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
      cv::Mat rgb_image = rgb_cv_ptr->image.clone();
      cv::Mat depth_image = depth_cv_ptr->image.clone();
      cv::Mat raw_depth_image;
      convert_32FC1_to_16UC1(raw_depth_image,depth_image);

      //perform object detection
      DetectionVector detections = compute(rgb_image,raw_depth_image);

      //Save timestamp (for publishers)
      _last_timestamp = logical_image_msg->header.stamp;

      //publish detections
      publishImageBoundingBoxes(detections);

      //publish label_image (for visualization only)
      sensor_msgs::ImagePtr label_image_msg = cv_bridge::CvImage(std_msgs::Header(),
                                                                 "bgr8",
                                                                 _label_image).toImageMsg();
      _label_image_pub.publish(label_image_msg);
    }
  }

  Eigen::Isometry3f LogicalDetectorNode::tfTransform2eigen(const tf::Transform &p){
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

  ModelVector LogicalDetectorNode::logicalImageToModels(const lucrezio_simulation_environments::LogicalImage::ConstPtr & logical_image_msg){
    int num_models = logical_image_msg->models.size();
    ModelVector models(num_models);
    Model temp_model;
    tf::StampedTransform model_pose;
    for(int i=0; i<num_models; ++i){
      temp_model._type=logical_image_msg->models[i].type;
      temp_model._min=Eigen::Vector3f(logical_image_msg->models[i].min.x,
                                      logical_image_msg->models[i].min.y,
                                      logical_image_msg->models[i].min.z);
      temp_model._max=Eigen::Vector3f(logical_image_msg->models[i].max.x,
                                      logical_image_msg->models[i].max.y,
                                      logical_image_msg->models[i].max.z);
      tf::poseMsgToTF(logical_image_msg->models[i].pose,model_pose);
      temp_model._pose=tfTransform2eigen(model_pose);

      models[i] = temp_model;
    }

    return models;
  }

  void LogicalDetectorNode::publishImageBoundingBoxes(const DetectionVector &detections){
    lucrezio_spme::ImageBoundingBoxesArray image_bounding_boxes;
    image_bounding_boxes.header.frame_id = "camera_depth_optical_frame";
    image_bounding_boxes.header.stamp = _last_timestamp;
    lucrezio_spme::ImageBoundingBox image_bounding_box;
    for(int i=0; i < detections.size(); ++i){
      image_bounding_box.type = detections[i]->type();
      image_bounding_box.top_left.r = detections[i]->topLeft().x();
      image_bounding_box.top_left.c = detections[i]->topLeft().y();
      image_bounding_box.bottom_right.r = detections[i]->bottomRight().x();
      image_bounding_box.bottom_right.c = detections[i]->bottomRight().y();
      lucrezio_spme::Pixel pixel;
      for(int j=0; j < detections[i]->pixels().size(); ++j){
        pixel.r = detections[i]->pixels()[j].x();
        pixel.c = detections[i]->pixels()[j].y();
        image_bounding_box.pixels.push_back(pixel);
      }
      image_bounding_boxes.image_bounding_boxes.push_back(image_bounding_box);
    }
    _image_bounding_boxes_pub.publish(image_bounding_boxes);
  }
}
