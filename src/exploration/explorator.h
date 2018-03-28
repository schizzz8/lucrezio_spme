#pragma once

#include <types/semantic_map.h>

#include <Eigen/Geometry>

//abstract class for exploration strategies
class Explorator{
public:
    Explorator();

    //this function implements the exploration strategy, takes as input the robot pose and the global map and determines where the robot should go next
    virtual void goToPose() = 0;

private:

    //global map
    SemanticMap _global_map;
    
    //robot pose
    Eigen::Isometry3f _globalT;
};

