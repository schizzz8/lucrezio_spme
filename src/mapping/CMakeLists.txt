add_library(lucrezio_mapping_library SHARED
  object.h object.cpp
  semantic_map.h
  base_mapper.h base_mapper.cpp
  semantic_mapper.h semantic_mapper.cpp
  semantic_mapper_node.h semantic_mapper_node.cpp
)

target_link_libraries(lucrezio_mapping_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

add_executable(semantic_mapper_node_app semantic_mapper_node_app.cpp)

target_link_libraries(semantic_mapper_node_app
  lucrezio_mapping_library
  ${catkin_LIBRARIES}
)
