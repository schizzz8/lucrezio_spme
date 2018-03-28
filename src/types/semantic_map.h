#pragma once

#include "types.hpp"

//this class is a container for the semantic map
class SemanticMap {
public:
  SemanticMap();
  virtual ~SemanticMap();

  //this function inserts an object into the map
  void addObject(Object* object_);

  //this function updates an object already present in the map
  void updateObject(Object* object_, int id);

  void clear();

private:

  //objects array that consitutes the map
  ObjectVector _objects;
};

