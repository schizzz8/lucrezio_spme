#pragma once

#include <core/semantic_map.h>
#include <core/detection.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class Mapper{
public:
    Mapper();

    virtual void extractObjects(const Detections &detections) = 0;
    virtual void findAssociations() = 0;
    virtual void mergeMaps() = 0;

private:
    SemanticMap _local_map;
    SemanticMap _global_map;
    Eigen::Isometry3f _globalT;

    std::vector<Association> _associations;
};

