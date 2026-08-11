add_library(mycamera myCamera.cpp)
target_include_directories(mycamera PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if(OpenCV_FOUND)
  target_include_directories(mycamera PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(mycamera PUBLIC ${OpenCV_LIBS})
else()
  target_compile_definitions(mycamera PUBLIC MY_CAMERA_FORCE_FAKE=1)
endif()

if(OpenCV_FOUND)
  add_executable(example ${PROJECT_SOURCE_DIR}/io/example.cpp)
  target_link_libraries(example PRIVATE mycamera)
endif()
