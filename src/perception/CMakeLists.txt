add_library(lucrezio_perception_library SHARED
  detection.h
  model.h
  base_detector.h
  logical_detector.h logical_detector.cpp
  logical_detector_node.h logical_detector_node.cpp
)

target_link_libraries(lucrezio_perception_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
