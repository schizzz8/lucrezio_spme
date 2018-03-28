#pragma once

#include <string>
#include <vector>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "detection.hpp"
#include "object.hpp"


typedef std::vector<Detection> DetectionVector;

typedef std::vector<Object*> ObjectVector;
typedef std::pair<Object,Object> Association;

