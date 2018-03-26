add_library(lucrezio_spme_mapping_library SHARED
  mapper.cpp mapper.h
)

target_link_libraries(lucrezio_spme_mapping_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
