#include "semantic_map.h"

SemanticMap::SemanticMap(){
  _objects.clear();
}

SemanticMap::~SemanticMap(){
  if (_objects.size()) {
    clear();
  }
}

void SemanticMap::addObject(Object *object_){
  _objects.push_back(object_);
}

void SemanticMap::clear(){
  for(int i=0; i<_objects.size(); ++i)
    delete _objects[i];

  _objects.clear();
}

