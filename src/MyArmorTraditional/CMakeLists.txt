add_library(armor_vision
  armor.cpp
  detector.cpp
  digit_recognizer.cpp
  tracker.cpp)
target_include_directories(armor_vision PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(armor_vision PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(armor_vision PUBLIC ${OpenCV_LIBS})
