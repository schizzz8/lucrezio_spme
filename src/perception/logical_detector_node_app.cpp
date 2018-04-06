#include <iostream>
#include "logical_detector_node.h"

using namespace lucrezio_spme;

int main(int argc, char **argv){

  ros::init(argc, argv, "logical_detector");
  ros::NodeHandle nh;

  LogicalDetectorNode detector(nh);

//  ros::spin();
  ros::Rate rate(4);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
