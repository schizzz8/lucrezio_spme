#pragma once

#include "object.h"

class SemanticMap {
public:
  SemanticMap();
  virtual ~SemanticMap();

  void addObject(Object* object_);
  void clear();

private:
  Objects _objects;
};

