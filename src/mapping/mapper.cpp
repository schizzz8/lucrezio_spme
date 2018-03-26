#include "mapper.h"

Mapper::Mapper(){
  _local_map.clear();
  _global_map.clear();
  _globalT = Eigen::Isometry3f::Identity();
  _associations.clear();
}
