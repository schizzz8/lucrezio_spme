add_library(lucrezio_spme_types_library SHARED
  semantic_map.cpp semantic_map.h
)

target_link_libraries(lucrezio_spme_types_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
