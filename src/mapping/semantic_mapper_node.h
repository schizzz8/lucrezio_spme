#pragma once

#include "semantic_mapper.h"

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include <lucrezio_spme/ImageBoundingBoxesArray.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetLinkState.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace lucrezio_spme{

  class SemanticMapperNode : public SemanticMapper{
  public:
    SemanticMapperNode(ros::NodeHandle nh_);

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg);

    void filterCallback(const lucrezio_spme::ImageBoundingBoxesArray::ConstPtr& image_bounding_boxes_msg,
                        const sensor_msgs::Image::ConstPtr& depth_image_msg);
  protected:
    ros::NodeHandle _nh;

    ros::Subscriber _camera_info_sub;
    bool _got_info;
    bool _first = true;

    //client for the get_link_state service
    ros::ServiceClient _link_state_client;

    message_filters::Subscriber<lucrezio_spme::ImageBoundingBoxesArray> _image_bounding_boxes_sub;
    message_filters::Subscriber<sensor_msgs::Image> _depth_image_sub;
    typedef message_filters::sync_policies::ApproximateTime<lucrezio_spme::ImageBoundingBoxesArray,sensor_msgs::Image> FilterSyncPolicy;
    message_filters::Synchronizer<FilterSyncPolicy> _synchronizer;

    uint32_t _shape;
    ros::Publisher _markers_pub;

    ros::Publisher _cloud_pub;

  private:

    //convert tf transform to eigen isometry
    Eigen::Isometry3f tfTransform2eigen(const tf::Transform& p);

    //converts ros message to detection vector
    DetectionVector imageBoundingBoxes2Detections(const lucrezio_spme::ImageBoundingBoxesArray::ConstPtr &image_bounding_boxes_msg);

    //utility function for publishing results
    void makeMarkerFromObject(visualization_msgs::Marker &marker, const ObjectPtr &object);
  };
}
