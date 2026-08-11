add_library(yolov5
  yolov5_utils.cpp
  yolov5.cpp)
target_include_directories(yolov5 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(yolov5 PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(yolov5 PUBLIC openvino::runtime ${OpenCV_LIBS})
