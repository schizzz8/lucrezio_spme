#pragma once

#include "semantic_map.h"
#include "detection.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <pcl/filters/statistical_outlier_removal.h>

class Mapper{
public:
    Mapper();

    void extractObjects(const Detections &detections);
    void findAssociations();
    void mergeMaps();

private:
    SemanticMap _local_map;
    SemanticMap _global_map;
    Eigen::Isometry3f _globalT;

    std::vector<Association> _associations;
};

