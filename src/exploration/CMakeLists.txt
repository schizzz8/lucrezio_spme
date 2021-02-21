add_library(lucrezio_spme_exploration_library SHARED
  explorator.cpp explorator.h
)

target_link_libraries(lucrezio_spme_exploration_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
