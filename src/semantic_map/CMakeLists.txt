add_library(lucrezio_semantic_map_library SHARED
  object.h object.cpp
  semantic_map.cpp semantic_map.h
)

target_link_libraries(lucrezio_semantic_map_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
