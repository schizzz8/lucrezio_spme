#include <iostream>
#include "semantic_mapper_node.h"

using namespace lucrezio_spme;

int main(int argc, char **argv){

  ros::init(argc, argv, "semantic_mapper");
  ros::NodeHandle nh;

  SemanticMapperNode mapper(nh);

  //  ros::spin();
  ros::Rate rate(4);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
