add_library(lucrezio_spme_perception_library SHARED
  detector.cpp detector.h
)

target_link_libraries(lucrezio_spme_perception_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
