#pragma once

#include <core/semantic_map.h>

#include <Eigen/Geometry>

class Explorator{
public:
    Explorator();

    virtual void goToPose() = 0;

private:
    SemanticMap _global_map;
    Eigen::Isometry3f _globalT;
};

