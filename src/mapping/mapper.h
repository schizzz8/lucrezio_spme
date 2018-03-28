#pragma once

#include <types/semantic_map.h>
#include <types/detection.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

//abstract class for semantic mapper
class Mapper{
public:
    Mapper();

    //this function takes as input the outcome of the object detector and builds a local map
    virtual void extractObjects(const DetectionVector &detections) = 0;
    
    //this function finds correspondences between objects in the local and the global map
    virtual void findAssociations() = 0;
    
    //this function updates the objects in the global map with the newly observed ones
    virtual void mergeMaps() = 0;

private:

    //map built from the current frame
    SemanticMap _local_map;
    
    //actual map that stores objects in a global reference frame and gets updated for each new observation
    SemanticMap _global_map;
    
    //pose of the robot w.r.t. the global map
    Eigen::Isometry3f _globalT;

    //this vector stores the output of the data-association
    std::vector<Association> _associations;
};

