add_library(lucrezio_mapping_library SHARED
  object.h object.cpp
  semantic_map.h semantic_map.cpp
  base_mapper.h base_mapper.cpp
  semantic_mapper.h semantic_mapper.cpp
  semantic_mapper_node.h semantic_mapper_node.cpp
)

target_link_libraries(lucrezio_mapping_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
