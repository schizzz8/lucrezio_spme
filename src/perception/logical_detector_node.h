#pragma once

#include <iostream>

#include <ros/ros.h>
#include <lucrezio_simulation_environments/LogicalImage.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <lucrezio_spme/ImageBoundingBoxesArray.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include "tf/tf.h"
#include "tf/transform_datatypes.h"

#include "logical_detector.h"

namespace lucrezio_spme{
  class LogicalDetectorNode : public LogicalDetector{
  public:

    //constructor
    LogicalDetectorNode(ros::NodeHandle nh_);

    //camera_info callback
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg);

    //synchronized subscriber callback
    void filterCallback(const lucrezio_simulation_environments::LogicalImage::ConstPtr &logical_image_msg,
                        const sensor_msgs::Image::ConstPtr &depth_image_msg,
                        const sensor_msgs::Image::ConstPtr &rgb_image_msg);
  protected:

    //node handle
    ros::NodeHandle _nh;

    //subscriber to camera_info topic
    ros::Subscriber _camera_info_sub;
    bool _got_info;

    //synchronized subscriber to rgbd frame and logical_image
    message_filters::Subscriber<lucrezio_simulation_environments::LogicalImage> _logical_image_sub;
    message_filters::Subscriber<sensor_msgs::Image> _depth_image_sub;
    message_filters::Subscriber<sensor_msgs::Image> _rgb_image_sub;
    typedef message_filters::sync_policies::ApproximateTime<lucrezio_simulation_environments::LogicalImage,
    sensor_msgs::Image,
    sensor_msgs::Image> FilterSyncPolicy;
    message_filters::Synchronizer<FilterSyncPolicy> _synchronizer;

    //publisher for the detections
    ros::Time _last_timestamp;
    ros::Publisher _image_bounding_boxes_pub;

    //publisher for the label image (visualization only)
    image_transport::ImageTransport _it;
    image_transport::Publisher _label_image_pub;

  private:

    //convert tf transform to eigen isometry
    Eigen::Isometry3f tfTransform2eigen(const tf::Transform& p);

    //extract models from logical image msg
    ModelVector logicalImageToModels(const lucrezio_simulation_environments::LogicalImage::ConstPtr &logical_image_msg);

    //publish detections
    void publishImageBoundingBoxes(const DetectionVector &detections);
  };
}
