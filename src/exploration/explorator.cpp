#include "explorator.h"

Explorator::Explorator(){
  _global_map.clear();
  _globalT = Eigen::Isometry3f::Identity();
}
