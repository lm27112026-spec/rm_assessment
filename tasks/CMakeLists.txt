add_executable(armor_demo armor_demo.cpp)
target_link_libraries(armor_demo PRIVATE armor_vision)

add_executable(serial_demo serial_demo.cpp)
target_link_libraries(serial_demo PRIVATE armor_vision mycamera communication)
if(MSVC)
  # Existing Chinese comments in headers may trigger MSVC C4819 codepage warnings.
  target_compile_options(serial_demo PRIVATE /wd4819)
endif()

add_executable(serial_loopback serial_loopback.cpp)
target_link_libraries(serial_loopback PRIVATE communication)

if(TARGET yolov5)
  add_executable(yolov5_armor_demo yolov5_armor_demo.cpp)
  target_link_libraries(yolov5_armor_demo PRIVATE yolov5 armor_vision)
  copy_yolo_runtime_dlls(yolov5_armor_demo)
endif()
